"""Lógica pura de bfmc_lane_detection: de una imagen de cámara al estado del carril.

No importa ROS. Cada imagen se procesa sola, sin memoria de las anteriores.

Pipeline:

1. Umbral de color: se aíslan las marcas blancas del carril (HSV).
2. Región de interés: se descarta la parte alta de la imagen (cielo, fondo).
3. Vista cenital: una transformación de perspectiva "aplana" el piso, así las
   líneas paralelas del carril quedan paralelas y las distancias son uniformes.
4. Ajuste: se buscan las líneas izquierda y derecha con ventanas deslizantes y se
   ajusta un polinomio de segundo grado a cada una.
5. Chequeos (A2-152): cada línea se descarta si tiene pocas ventanas con píxeles, si el
   polinomio ajusta mal (residuo) o si es más curva que la curva más cerrada de la pista.
   Si las dos pasan pero no están a un ancho de carril entre sí (medido perpendicular a
   las líneas, en varias alturas), se queda la que mejor ajusta.
6. Medición: a una fila de referencia se calculan el desvío lateral y el error de
   rumbo, en metros y radianes.

Convenciones (las de bfmc_interfaces): unidades SI, y un desvío o ángulo positivo
es hacia la izquierda del auto.

El detector no tiene memoria: el fallback cuando no hay líneas válidas (repetir el último
estado, perder el carril) está en lane_tracker.py.

`detect` devuelve solo el resultado. `analizar` hace lo mismo pero además devuelve lo
que el detector vio por el camino (la máscara de blancos, las ventanas, las curvas),
para dibujarlo en las imágenes de depuración.
"""
from dataclasses import dataclass
import math
from typing import NamedTuple, Optional, Sequence

import cv2
import numpy as np

# Un cuadrilátero de warp_src con un giro menor a esto (producto vectorial de dos lados
# consecutivos, en fracciones de la imagen) se toma por degenerado o cruzado.
GIRO_MINIMO_WARP = 1e-4


def cuadrilatero_valido(warp_src: Sequence[float]) -> bool:
    """Indica si los 4 puntos forman un cuadrilátero convexo en el orden esperado.

    El orden es arriba-izquierda, arriba-derecha, abajo-derecha, abajo-izquierda: en la
    imagen (con y hacia abajo) cada giro entre lados consecutivos es en el mismo sentido.
    Un trapecio degenerado (puntos alineados) o cruzado no se puede usar como
    transformación de perspectiva.
    """
    puntos = np.array(warp_src, dtype=np.float64).reshape(4, 2)
    lados = np.roll(puntos, -1, axis=0) - puntos
    giros = lados[:, 0] * np.roll(lados[:, 1], -1) - lados[:, 1] * np.roll(lados[:, 0], -1)
    return bool(np.all(giros > GIRO_MINIMO_WARP))


@dataclass(frozen=True)
class LaneConfig:
    """Parámetros de la visión. Sus valores vienen de config/params.yaml."""

    white_hsv_min: Sequence[int]
    white_hsv_max: Sequence[int]
    morph_kernel_px: int
    roi_top_fraction: float
    warp_src: Sequence[float]
    warp_width: int
    warp_height: int
    lane_width_m: float       # ancho real del carril (dato de la pista)
    view_width_m: float       # ancho del piso que cubre la vista cenital (dato de la cámara)
    view_length_m: float      # largo hacia adelante que cubre la vista cenital
    n_windows: int
    window_margin_px: int
    min_pixels: int
    # Chequeos de cada línea y del par (A2-152).
    min_windows: int                    # ventanas con píxeles que necesita una línea
    max_residual_m: float               # error medio del polinomio [m]
    min_curve_radius_m: float           # radio de la curva más cerrada de la pista [m]
    lane_width_tolerance_m: float       # cuánto puede diferir el ancho medido de lane_width_m
    width_check_fractions: Sequence[float]  # alturas (0 = cerca, 1 = lejos) donde se mide
    reference_row_fraction: float
    full_confidence_pixels: int
    intersection_band_top_fraction: float
    intersection_row_fraction: float
    intersection_min_rows: int

    def __post_init__(self):
        if len(self.white_hsv_min) != 3 or len(self.white_hsv_max) != 3:
            raise ValueError('white_hsv_min y white_hsv_max deben tener 3 valores (H, S, V)')
        if any(lo > hi for lo, hi in zip(self.white_hsv_min, self.white_hsv_max)):
            raise ValueError('cada valor de white_hsv_min debe ser <= al de white_hsv_max')
        if self.morph_kernel_px < 1:
            raise ValueError('morph_kernel_px debe ser al menos 1')
        if not 0.0 <= self.roi_top_fraction < 1.0:
            raise ValueError('roi_top_fraction debe estar en [0, 1)')
        if len(self.warp_src) != 8 or not all(0.0 <= v <= 1.0 for v in self.warp_src):
            raise ValueError('warp_src debe tener 8 valores entre 0 y 1: x, y de 4 puntos')
        if not cuadrilatero_valido(self.warp_src):
            raise ValueError('warp_src no forma un cuadrilátero válido: los puntos van en el '
                             'orden arriba-izquierda, arriba-derecha, abajo-derecha, '
                             'abajo-izquierda, sin cruzarse ni quedar alineados')
        if self.warp_width < 2 or self.warp_height < 2:
            raise ValueError('warp_width y warp_height deben ser al menos 2')
        if min(self.lane_width_m, self.view_width_m, self.view_length_m) <= 0.0:
            raise ValueError('lane_width_m, view_width_m y view_length_m deben ser positivos')
        if self.lane_width_m >= self.view_width_m:
            raise ValueError('el carril (%.2f m) no entra en la vista cenital (%.2f m de ancho): '
                             'las dos líneas tienen que verse' % (
                                 self.lane_width_m, self.view_width_m))
        if self.n_windows < 1 or self.window_margin_px < 1 or self.min_pixels < 1:
            raise ValueError('n_windows, window_margin_px y min_pixels deben ser positivos')
        if not 1 <= self.min_windows <= self.n_windows:
            raise ValueError('min_windows debe estar entre 1 y n_windows')
        if min(self.max_residual_m, self.min_curve_radius_m, self.lane_width_tolerance_m) <= 0:
            raise ValueError('max_residual_m, min_curve_radius_m y lane_width_tolerance_m '
                             'deben ser positivos')
        if not self.width_check_fractions or not all(
                0.0 <= f <= 1.0 for f in self.width_check_fractions):
            raise ValueError('width_check_fractions debe tener al menos un valor entre 0 y 1')
        if not 0.0 <= self.reference_row_fraction < 1.0:
            raise ValueError('reference_row_fraction debe estar en [0, 1)')
        if self.full_confidence_pixels < 1:
            raise ValueError('full_confidence_pixels debe ser al menos 1')
        if not 0.0 <= self.intersection_band_top_fraction < 1.0:
            raise ValueError('intersection_band_top_fraction debe estar en [0, 1)')
        if not 0.0 < self.intersection_row_fraction <= 1.0 or self.intersection_min_rows < 1:
            raise ValueError('intersection_row_fraction debe estar en (0, 1] y '
                             'intersection_min_rows ser al menos 1')


@dataclass(frozen=True)
class LaneResult:
    """Estado del carril en una imagen."""

    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    confidence: float
    intersection_detected: bool
    left_valid: bool = False
    right_valid: bool = False
    failed_checks: str = ''      # qué chequeos fallaron, para la depuración ('' si ninguno)


class Linea(NamedTuple):
    """Una línea ajustada en la vista cenital: x = a y² + b y + c, en píxeles."""

    coef: np.ndarray
    pixeles: int
    ventanas: int          # ventanas en las que se encontraron píxeles
    residuo_px: float      # error medio |x ajustada - x| de los píxeles


NOT_DETECTED = LaneResult(False, 0.0, 0.0, 0.0, False)


@dataclass
class Analisis:
    """Lo que el detector vio en una imagen, para dibujarlo.

    `izquierda` y `derecha` son None o la `Linea` encontrada (pase o no los chequeos) en la
    vista cenital. `trazas` guarda, para cada lado ('izq' y 'der'), la columna donde
    arranca la línea y las ventanas probadas: (x_ini, y_ini, x_fin, y_fin, píxeles).
    `centro` es None o el diccionario con el que se midió el carril.
    """

    resultado: LaneResult
    mascara: np.ndarray
    cenital: np.ndarray
    izquierda: Optional[tuple]
    derecha: Optional[tuple]
    trazas: dict
    centro: Optional[dict]


class LaneDetector:
    """Estima el estado del carril a partir de una imagen BGR."""

    def __init__(self, config: LaneConfig):
        self._config = config
        self._matrices = {}
        # La escala es solo de la cámara: qué parte del piso cubre la vista. El ancho del
        # carril es un dato de la pista y no la cambia.
        self._mpp_x = config.view_width_m / config.warp_width
        self._mpp_y = config.view_length_m / config.warp_height
        self._ancho_carril_px = config.lane_width_m / self._mpp_x

    @property
    def config(self) -> LaneConfig:
        """La configuración con la que se creó el detector."""
        return self._config

    @property
    def lane_width_px(self) -> float:
        """Ancho del carril en píxeles de la vista cenital (calculado, no un parámetro)."""
        return self._ancho_carril_px

    # --- geometría ----------------------------------------------------------------
    def warp_matrix(self, width: int, height: int):
        """Matriz de perspectiva de la imagen de cámara a la vista cenital."""
        clave = (width, height)
        if clave not in self._matrices:
            cfg = self._config
            origen = np.array(cfg.warp_src, dtype=np.float32).reshape(4, 2)
            origen = origen * np.array([width, height], dtype=np.float32)
            destino = np.array(
                [[0, 0], [cfg.warp_width, 0], [cfg.warp_width, cfg.warp_height],
                 [0, cfg.warp_height]], dtype=np.float32)
            self._matrices[clave] = cv2.getPerspectiveTransform(origen, destino)
        return self._matrices[clave]

    # --- detección ------------------------------------------------------------------
    def detect(self, bgr: np.ndarray) -> LaneResult:
        """Devuelve el estado del carril. `bgr` es una imagen alto x ancho x 3 de uint8."""
        return self._procesar(bgr, None).resultado

    def analizar(self, bgr: np.ndarray) -> Analisis:
        """Igual que `detect`, pero devuelve además lo que el detector vio."""
        return self._procesar(bgr, {})

    def _procesar(self, bgr, trazas):
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('se esperaba una imagen BGR de alto x ancho x 3')
        alto, ancho = bgr.shape[:2]
        mascara = self._mascara_blanca(bgr)
        cenital = self._vista_cenital(mascara, ancho, alto)

        izquierda, derecha = self._buscar_lineas(cenital, trazas)
        interseccion = self._hay_interseccion(cenital)
        izquierda_ok, derecha_ok, fallos = self._validar(izquierda, derecha)
        centro = self._centro(izquierda_ok, derecha_ok)
        validas = {'left_valid': izquierda_ok is not None, 'right_valid': derecha_ok is not None,
                   'failed_checks': '; '.join(fallos)}
        if centro is None:
            resultado = LaneResult(False, 0.0, 0.0, 0.0, interseccion, **validas)
        else:
            lateral, rumbo, confianza = self._medir(centro)
            resultado = LaneResult(True, lateral, rumbo, confianza, interseccion, **validas)
        return Analisis(resultado, mascara, cenital, izquierda, derecha,
                        trazas if trazas is not None else {}, centro)

    def _mascara_blanca(self, bgr):
        cfg = self._config
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mascara = cv2.inRange(hsv, np.array(cfg.white_hsv_min, dtype=np.uint8),
                              np.array(cfg.white_hsv_max, dtype=np.uint8))
        mascara[:int(bgr.shape[0] * cfg.roi_top_fraction)] = 0
        if cfg.morph_kernel_px > 1:
            nucleo = np.ones((cfg.morph_kernel_px, cfg.morph_kernel_px), dtype=np.uint8)
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
        return mascara

    def _vista_cenital(self, mascara, ancho, alto):
        cfg = self._config
        return cv2.warpPerspective(
            mascara, self.warp_matrix(ancho, alto), (cfg.warp_width, cfg.warp_height),
            flags=cv2.INTER_NEAREST)

    # --- líneas ---------------------------------------------------------------------
    def _buscar_lineas(self, cenital, trazas=None):
        """Devuelve (ajuste izquierdo, ajuste derecho). Cada uno es None o (coef, píxeles).

        Si `trazas` es un diccionario, se completa con la columna de arranque y las
        ventanas de cada lado.
        """
        cfg = self._config
        ancho = cenital.shape[1]
        histograma = (cenital[cenital.shape[0] // 2:] > 0).sum(axis=0)
        medio = ancho // 2

        base_izquierda = self._base(histograma[:medio], 0)
        base_derecha = self._base(histograma[medio:], medio)
        ajustes = []
        for lado, base in (('izq', base_izquierda), ('der', base_derecha)):
            ventanas = [] if trazas is not None else None
            ajustes.append(None if base is None else self._ajustar(cenital, base, cfg, ventanas))
            if trazas is not None:
                trazas[lado] = {'base': base, 'ventanas': ventanas}
        return ajustes[0], ajustes[1]

    def _base(self, histograma, desplazamiento):
        if histograma.size == 0 or histograma.max() < self._config.min_pixels:
            return None
        return int(np.argmax(histograma)) + desplazamiento

    @staticmethod
    def _ajustar(cenital, base, cfg, ventanas=None):
        alto, ancho = cenital.shape
        paso = alto // cfg.n_windows
        x_actual = base
        xs, ys = [], []
        con_pixeles = 0
        for ventana in range(cfg.n_windows):
            y_fin = alto - ventana * paso
            y_ini = max(0, y_fin - paso)
            x_ini = max(0, x_actual - cfg.window_margin_px)
            x_fin = min(ancho, x_actual + cfg.window_margin_px)
            fy, fx = np.nonzero(cenital[y_ini:y_fin, x_ini:x_fin])
            if ventanas is not None:
                ventanas.append((x_ini, y_ini, x_fin, y_fin, int(fy.size)))
            con_pixeles += int(fy.size > 0)
            if fy.size >= cfg.min_pixels:
                x_actual = int(fx.mean()) + x_ini
            xs.extend((fx + x_ini).tolist())
            ys.extend((fy + y_ini).tolist())

        if len(ys) < cfg.min_pixels or len(set(ys)) < 3:
            return None
        y = np.array(ys, dtype=np.float64)
        x = np.array(xs, dtype=np.float64)
        coeficientes = np.polyfit(y, x, 2)
        residuo = float(np.mean(np.abs(np.polyval(coeficientes, y) - x)))
        return Linea(coeficientes, len(ys), con_pixeles, residuo)

    # --- medición -------------------------------------------------------------------
    def _centro(self, izquierda, derecha) -> Optional[dict]:
        """Decide dónde está el centro del carril en la fila de referencia.

        Recibe solo líneas que pasaron los chequeos. Devuelve None si no hay ninguna. Si
        no, un diccionario con la columna del centro (`centro_px`), la pendiente de la
        curva (`pendiente`), los píxeles que la respaldan, la confianza base, la fila
        (`y_px`) y qué líneas se usaron (`modo`: 'ambas', 'izquierda' o 'derecha').
        """
        cfg = self._config
        if izquierda is None and derecha is None:
            return None

        y = cfg.warp_height * (1.0 - cfg.reference_row_fraction)
        ancho_carril_px = self._ancho_carril_px

        def evaluar(ajuste):
            a, b, c = ajuste[0]
            return a * y * y + b * y + c, 2.0 * a * y + b

        # Las líneas ya pasaron los chequeos (ver _validar): si llegan las dos, son un carril.
        if izquierda is not None and derecha is not None:
            x_izq, pendiente_izq = evaluar(izquierda)
            x_der, pendiente_der = evaluar(derecha)
            centro = 0.5 * (x_izq + x_der)
            pendiente = 0.5 * (pendiente_izq + pendiente_der)
            pixeles = izquierda[1] + derecha[1]
            base_confianza = 1.0
            modo = 'ambas'
        else:
            ajuste, signo = (izquierda, +1.0) if izquierda is not None else (derecha, -1.0)
            x_linea, pendiente = evaluar(ajuste)
            centro = x_linea + signo * 0.5 * ancho_carril_px
            pixeles = ajuste[1]
            base_confianza = 0.5
            modo = 'izquierda' if signo > 0 else 'derecha'
        return {'centro_px': centro, 'pendiente': pendiente, 'pixeles': pixeles,
                'base_confianza': base_confianza, 'y_px': y, 'modo': modo}

    def _validar(self, izquierda, derecha):
        """Aplica los chequeos. Devuelve (izquierda válida, derecha válida, fallos).

        Cada una es la Linea o None. Una línea que no pasa sus chequeos se descarta. Si las
        dos pasan pero no están a un ancho de carril entre sí, se descarta la que ajusta peor
        (mayor residuo; a igual residuo, la de menos píxeles): una mancha de reflejo puede
        tener muchos píxeles, pero no tiene forma de línea.
        """
        fallos = []
        validas = []
        for nombre, linea in (('izq', izquierda), ('der', derecha)):
            motivo = None if linea is None else self._fallo_de_linea(linea)
            if motivo is not None:
                fallos.append('%s: %s' % (nombre, motivo))
            validas.append(None if motivo is not None else linea)
        izquierda, derecha = validas
        if izquierda is not None and derecha is not None and not self._ancho_ok(
                izquierda, derecha):
            fallos.append('ancho de carril')
            if (izquierda.residuo_px, -izquierda.pixeles) > (derecha.residuo_px,
                                                              -derecha.pixeles):
                izquierda = None
            else:
                derecha = None
        return izquierda, derecha, fallos

    def _fallo_de_linea(self, linea: Linea) -> Optional[str]:
        """El motivo por el que la línea no pasa sus chequeos, o None si los pasa."""
        cfg = self._config
        if linea.ventanas < cfg.min_windows:
            return 'pocas ventanas (%d)' % linea.ventanas
        if linea.residuo_px * self._mpp_x > cfg.max_residual_m:
            return 'residuo %.3f m' % (linea.residuo_px * self._mpp_x)
        if self._curvatura_m(linea, self._fila_de_referencia()) > 1.0 / cfg.min_curve_radius_m:
            return 'demasiado curva'
        return None

    def _fila_de_referencia(self):
        return self._config.warp_height * (1.0 - self._config.reference_row_fraction)

    def _pendiente_m(self, linea: Linea, y_px: float) -> float:
        """dx/dy en metros (la escala puede ser distinta en cada eje)."""
        a, b, _ = linea.coef
        return (2.0 * a * y_px + b) * self._mpp_x / self._mpp_y

    def _curvatura_m(self, linea: Linea, y_px: float) -> float:
        """|curvatura| [1/m] de la línea en la fila `y_px`."""
        a_m = linea.coef[0] * self._mpp_x / (self._mpp_y ** 2)    # d²x/dy² en metros
        pendiente = self._pendiente_m(linea, y_px)
        return abs(2.0 * a_m) / (1.0 + pendiente ** 2) ** 1.5

    def _ancho_ok(self, izquierda: Linea, derecha: Linea) -> bool:
        """Las dos líneas están a un ancho de carril, medido perpendicular a ellas.

        En la misma fila, la distancia horizontal entre dos líneas paralelas inclinadas un
        ángulo θ es ancho / cos θ: en una curva con θ = 30°, un carril de 0,37 m mide 0,43 m
        en horizontal. Por eso se corrige por cos θ antes de comparar.
        """
        cfg = self._config
        for fraccion in cfg.width_check_fractions:
            y = cfg.warp_height * (1.0 - fraccion)
            horizontal = (np.polyval(derecha.coef, y) - np.polyval(izquierda.coef, y))
            angulo = math.atan(0.5 * (self._pendiente_m(izquierda, y)
                                      + self._pendiente_m(derecha, y)))
            ancho = horizontal * self._mpp_x * math.cos(angulo)
            if abs(ancho - cfg.lane_width_m) > cfg.lane_width_tolerance_m:
                return False
        return True

    def _medir(self, centro: dict) -> tuple:
        cfg = self._config
        lateral = (cfg.warp_width / 2.0 - centro['centro_px']) * self._mpp_x
        rumbo = math.atan(centro['pendiente'] * self._mpp_x / self._mpp_y)
        confianza = centro['base_confianza'] * min(
            1.0, centro['pixeles'] / cfg.full_confidence_pixels)
        return lateral, rumbo, confianza

    # --- intersección ---------------------------------------------------------------
    def _hay_interseccion(self, cenital) -> bool:
        """Una intersección es una franja horizontal blanca que cruza buena parte de la vista."""
        cfg = self._config
        banda = cenital[int(cenital.shape[0] * cfg.intersection_band_top_fraction):]
        if banda.size == 0:
            return False
        fraccion_por_fila = (banda > 0).mean(axis=1)
        filas = int((fraccion_por_fila >= cfg.intersection_row_fraction).sum())
        return filas >= cfg.intersection_min_rows
