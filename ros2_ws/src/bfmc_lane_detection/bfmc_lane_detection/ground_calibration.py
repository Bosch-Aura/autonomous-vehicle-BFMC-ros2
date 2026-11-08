"""Calibración y geometría de la vista cenital. Lógica pura.

No importa ROS ni lee el reloj.

La geometría del detector sale SIEMPRE de tres datos independientes, nunca a mano:

- la pose de la cámara respecto del piso (altura, pitch y roll), que se mide con la
  profundidad (`medir_pose`);
- los intrínsecos de la cámara (los de su camera_info);
- el rectángulo del piso que se quiere ver (`GroundView`, en metros).

Con eso, `trapecio` calcula `warp_src` y la escala es la del rectángulo: así la escala
queda atada al trapecio y no hay ningún número que se pueda desincronizar. El ancho del
carril es un dato de la pista y no interviene (ver `lane_config_desde_parametros`).

Pasos:

1. Puntos 3D del piso: de la mitad de abajo de la imagen, los píxeles con profundidad
   entre `depth_min_m` y `depth_max_m`, promediados entre los cuadros y submuestreados.
2. Plano con RANSAC (semilla fija, resultado reproducible) y reajuste por mínimos
   cuadrados sobre los inliers: n · P + d = 0, con |n| = 1 y d > 0 (n apunta hacia
   arriba, del lado de la cámara).
3. Altura h = d, pie de la cámara en el piso O = -d n, eje "adelante" f = el eje óptico
   proyectado sobre el piso, eje "izquierda" l = n x f.
4. Un rectángulo del piso de medidas conocidas (`near_m` a `far_m` adelante,
   ± `half_width_m` a los lados) se proyecta a la imagen: ese es el trapecio.
5. La escala sale de las medidas del rectángulo.

Pitch y roll alcanzan para reconstruir el plano (el giro alrededor del eje vertical no
cambia el plano): n = (sen r cos p, -cos r cos p, -sen p). Por eso basta guardar la
altura, el pitch y el roll de la última calibración para volver a proyectar el trapecio
sin medir de nuevo.

Coordenadas de la cámara: x a la derecha, y hacia abajo, z hacia adelante (eje óptico).

Lo que la profundidad NO mide: el giro de la cámara alrededor del eje vertical (yaw) y su
corrimiento lateral respecto del eje del auto. La calibración asume que la cámara mira
hacia adelante del auto y está centrada en él.
"""
import dataclasses
from dataclasses import dataclass
import math
from typing import Optional, Sequence

from bfmc_lane_detection.lane_detection_logic import cuadrilatero_valido, LaneConfig
import numpy as np

ESQUINAS = ('arriba-izquierda', 'arriba-derecha', 'abajo-derecha', 'abajo-izquierda')
DECIMALES_WARP = 4
# La pose se guarda con 4 decimales: 0,00005° de pitch mueven el trapecio menos de 0,01 px.
DECIMALES_POSE = 4
# RANSAC evalúa los candidatos de a bloques: la matriz puntos x candidatos entera puede pesar
# cientos de MB con calib_subsample = 1. Con 16 candidatos por bloque, unos 20 MB como máximo.
CANDIDATOS_POR_BLOQUE = 16


class CalibrationError(ValueError):
    """La calibración no es válida. El mensaje dice por qué y qué hacer, en castellano."""


@dataclass(frozen=True)
class Intrinsics:
    """Intrínsecos de la imagen de profundidad alineada al color (los de su camera_info)."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class GroundCalibrationConfig:
    """Parámetros de la calibración. Sus valores vienen de config/params.yaml."""

    depth_min_m: float
    depth_max_m: float
    subsample: int
    min_points: int
    ransac_iters: int
    ransac_seed: int
    inlier_m: float
    min_inlier_fraction: float
    height_range_m: Sequence[float]
    pitch_range_deg: Sequence[float]

    def __post_init__(self):
        if not 0.0 < self.depth_min_m < self.depth_max_m:
            raise ValueError('calib_depth_min_m y calib_depth_max_m deben cumplir '
                             '0 < mínimo < máximo')
        if self.subsample < 1 or self.min_points < 3 or self.ransac_iters < 1:
            raise ValueError('calib_subsample y calib_ransac_iters deben ser al menos 1, y '
                             'calib_min_points al menos 3')
        if self.inlier_m <= 0.0 or not 0.0 < self.min_inlier_fraction <= 1.0:
            raise ValueError('calib_inlier_m debe ser positivo y calib_min_inlier_fraction '
                             'estar en (0, 1]')
        for nombre, rango in (('calib_height_range_m', self.height_range_m),
                              ('calib_pitch_range_deg', self.pitch_range_deg)):
            if len(rango) != 2 or rango[0] >= rango[1]:
                raise ValueError('%s debe ser [mínimo, máximo] con mínimo < máximo' % nombre)


@dataclass(frozen=True)
class GroundView:
    """El rectángulo del piso que muestra la vista cenital, medido desde el pie de la cámara."""

    near_m: float             # borde de abajo de la vista
    far_m: float              # borde de arriba
    half_width_m: float       # mitad del ancho

    def __post_init__(self):
        if not 0.0 <= self.near_m < self.far_m or self.half_width_m <= 0.0:
            raise ValueError('hace falta 0 <= calib_forward_near_m < calib_forward_far_m y '
                             'calib_half_width_m positivo')

    @property
    def width_m(self) -> float:
        return 2.0 * self.half_width_m

    @property
    def length_m(self) -> float:
        return self.far_m - self.near_m


@dataclass(frozen=True)
class GroundPose:
    """Dónde está la cámara respecto del piso."""

    height_m: float           # altura del centro óptico sobre el piso
    pitch_deg: float          # positivo = mirando hacia abajo
    roll_deg: float           # giro alrededor del eje óptico; positivo = lado derecho más alto
    inlier_fraction: Optional[float] = None   # solo si se midió con la profundidad

    @property
    def normal(self) -> np.ndarray:
        """Normal del piso (hacia arriba) en coordenadas de la cámara."""
        p, r = math.radians(self.pitch_deg), math.radians(self.roll_deg)
        return np.array([math.sin(r) * math.cos(p), -math.cos(r) * math.cos(p), -math.sin(p)])


@dataclass(frozen=True)
class GroundCalibration:
    """El resultado completo: la pose y los intrínsecos medidos, y la geometría que dan."""

    pose: GroundPose
    intrinsics: Intrinsics
    warp_src: tuple
    mm_per_px_x: float
    mm_per_px_y: float

    def resumen(self) -> str:
        """Lo que se midió, en una línea."""
        if abs(self.mm_per_px_x - self.mm_per_px_y) < 0.005:
            escala = '1 px = %.2f mm' % self.mm_per_px_x
        else:
            escala = '1 px = %.2f mm de ancho x %.2f mm de largo' % (
                self.mm_per_px_x, self.mm_per_px_y)
        return 'altura %.3f m, pitch %.1f°, roll %.1f°, inliers %d%%, %s' % (
            self.pose.height_m, self.pose.pitch_deg + 0.0, self.pose.roll_deg + 0.0,
            round(100 * (self.pose.inlier_fraction or 0.0)), escala)

    def parametros(self) -> dict:
        """Los parámetros del nodo que fija esta calibración."""
        intr = self.intrinsics
        return {
            'calib_height_m': round(self.pose.height_m, DECIMALES_POSE),
            'calib_pitch_deg': round(self.pose.pitch_deg, DECIMALES_POSE),
            'calib_roll_deg': round(self.pose.roll_deg, DECIMALES_POSE),
            'camera_intrinsics': [float(intr.fx), float(intr.fy), float(intr.cx),
                                  float(intr.cy), float(intr.width), float(intr.height)],
        }


# --- pasos --------------------------------------------------------------------------------

def promediar(profundidades: Sequence[np.ndarray]) -> np.ndarray:
    """Promedia por píxel varios cuadros en metros. 0 o NaN es "sin dato" y no cuenta."""
    if not profundidades:
        raise CalibrationError('no llegó ninguna imagen de profundidad')
    if len({np.shape(p) for p in profundidades}) > 1:
        raise CalibrationError('llegaron imágenes de profundidad de distintos tamaños: '
                               'revisá que no haya dos cámaras publicando en el mismo topic')
    pila = np.stack([np.asarray(p, dtype=np.float64) for p in profundidades])
    pila[~np.isfinite(pila) | (pila <= 0.0)] = np.nan
    validos = np.isfinite(pila).sum(axis=0)
    suma = np.nansum(pila, axis=0)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(validos > 0, suma / np.maximum(validos, 1), np.nan)


def puntos_del_piso(profundidad: np.ndarray, intr: Intrinsics,
                    cfg: GroundCalibrationConfig) -> np.ndarray:
    """Puntos 3D (N x 3, en metros) candidatos a piso: la mitad de abajo de la imagen."""
    alto, ancho = profundidad.shape
    if (ancho, alto) != (intr.width, intr.height):
        raise CalibrationError('la profundidad (%dx%d) no coincide con su camera_info (%dx%d)'
                               % (ancho, alto, intr.width, intr.height))
    paso = cfg.subsample
    v, u = np.mgrid[alto // 2:alto:paso, 0:ancho:paso]
    z = profundidad[alto // 2:alto:paso, 0:ancho:paso]
    usar = np.isfinite(z) & (z >= cfg.depth_min_m) & (z <= cfg.depth_max_m)
    z, u, v = z[usar], u[usar], v[usar]
    return np.column_stack(((u - intr.cx) * z / intr.fx, (v - intr.cy) * z / intr.fy, z))


def ajustar_plano(puntos: np.ndarray, cfg: GroundCalibrationConfig):
    """RANSAC y reajuste. Devuelve (n, d, fracción de inliers) con |n| = 1 y d > 0."""
    if len(puntos) < cfg.min_points:
        raise CalibrationError(
            'hay %d puntos de piso con profundidad válida y hacen falta %d: el piso no se ve. '
            'Revisá que la cámara mire al piso y que no haya nada tapándola'
            % (len(puntos), cfg.min_points))

    azar = np.random.default_rng(cfg.ransac_seed)
    muestras = puntos[azar.integers(0, len(puntos), size=(cfg.ransac_iters, 3))]
    normales = np.cross(muestras[:, 1] - muestras[:, 0], muestras[:, 2] - muestras[:, 0])
    largos = np.linalg.norm(normales, axis=1)
    sanas = largos > 1e-9
    if not sanas.any():
        raise CalibrationError('los puntos de profundidad están todos alineados: el piso no '
                               'se ve bien. Despejá la zona delante de la cámara')
    normales = normales[sanas] / largos[sanas, None]
    distancias = -np.einsum('ij,ij->i', normales, muestras[sanas, 0])
    conteos = np.concatenate([
        (np.abs(puntos @ normales[i:i + CANDIDATOS_POR_BLOQUE].T
                + distancias[i:i + CANDIDATOS_POR_BLOQUE]) < cfg.inlier_m).sum(axis=0)
        for i in range(0, len(normales), CANDIDATOS_POR_BLOQUE)])
    mejor = int(np.argmax(conteos))
    inliers = puntos[np.abs(puntos @ normales[mejor] + distancias[mejor]) < cfg.inlier_m]

    # Reajuste: el plano que mejor pasa por los inliers es el de menor varianza.
    centro = inliers.mean(axis=0)
    normal = np.linalg.svd(inliers - centro, full_matrices=False)[2][-1]
    d = -float(normal @ centro)
    if d < 0.0:
        normal, d = -normal, -d
    fraccion = float((np.abs(puntos @ normal + d) < cfg.inlier_m).mean())
    return normal, d, fraccion


def medir_pose(profundidades: Sequence[np.ndarray], intr: Intrinsics,
               cfg: GroundCalibrationConfig) -> GroundPose:
    """Altura, pitch y roll de la cámara respecto del piso, ya validados."""
    puntos = puntos_del_piso(promediar(profundidades), intr, cfg)
    normal, d, fraccion = ajustar_plano(puntos, cfg)
    if fraccion < cfg.min_inlier_fraction:
        raise CalibrationError(
            'solo el %d%% de los puntos está en el piso (hace falta %d%%): hay algo delante '
            'de la cámara o el piso no se ve. Despejá la zona'
            % (round(100 * fraccion), round(100 * cfg.min_inlier_fraction)))

    pitch = math.degrees(math.asin(float(np.clip(-normal[2], -1.0, 1.0))))
    roll = math.degrees(math.atan2(float(normal[0]), float(-normal[1])))
    pose = GroundPose(float(d), pitch, roll, fraccion)

    minimo, maximo = cfg.height_range_m
    if not minimo <= d <= maximo:
        raise CalibrationError(
            'la altura medida (%.3f m) no tiene sentido: tiene que estar entre %.2f y %.2f m. '
            'Probablemente el plano encontrado no es el piso: despejá la zona'
            % (d, minimo, maximo))
    minimo, maximo = cfg.pitch_range_deg
    if not minimo <= pitch <= maximo:
        raise CalibrationError(
            'la inclinación medida (%.1f°) no tiene sentido: tiene que estar entre %.0f° y '
            '%.0f°. Probablemente el plano encontrado es una pared o un objeto, no el piso: '
            'despejá la zona' % (pitch, minimo, maximo))
    return pose


def ejes_del_piso(pose: GroundPose):
    """(O, f, l): pie de la cámara en el piso, eje adelante y eje izquierda, unitarios."""
    normal = pose.normal
    z = np.array([0.0, 0.0, 1.0])
    adelante = z - (z @ normal) * normal
    largo = np.linalg.norm(adelante)
    if largo < 1e-6:
        raise CalibrationError('la cámara mira derecho al piso: no se puede saber dónde es '
                               'adelante')
    adelante /= largo
    izquierda = np.cross(normal, adelante)
    if izquierda[0] > 0.0:          # en la cámara +x es la derecha
        izquierda = -izquierda
    return -pose.height_m * normal, adelante, izquierda


def punto_del_piso(pose: GroundPose, adelante_m: float, izquierda_m: float) -> np.ndarray:
    """Punto 3D (en la cámara) del piso a esa distancia adelante y a la izquierda."""
    origen, adelante, izquierda = ejes_del_piso(pose)
    return origen + adelante_m * adelante + izquierda_m * izquierda


def a_la_imagen(punto: np.ndarray, intr: Intrinsics):
    """Píxel (u, v) de un punto 3D, o None si está detrás de la cámara."""
    if punto[2] <= 1e-6:
        return None
    return (intr.fx * punto[0] / punto[2] + intr.cx, intr.fy * punto[1] / punto[2] + intr.cy)


def trapecio(pose: GroundPose, intr: Intrinsics, vista: GroundView) -> tuple:
    """`warp_src`: las esquinas del rectángulo del piso en la imagen, como fracciones.

    El orden es el de warp_src: arriba-izquierda, arriba-derecha, abajo-derecha,
    abajo-izquierda ("arriba" = lejos). Lanza CalibrationError si no entra en la imagen.
    """
    cerca, lejos, medio = vista.near_m, vista.far_m, vista.half_width_m
    esquinas = ((lejos, medio), (lejos, -medio), (cerca, -medio), (cerca, medio))
    warp_src = []
    for nombre, (x, y) in zip(ESQUINAS, esquinas):
        pixel = a_la_imagen(punto_del_piso(pose, x, y), intr)
        fuera = pixel is None or not (0.0 <= pixel[0] <= intr.width
                                      and 0.0 <= pixel[1] <= intr.height)
        if fuera:
            raise CalibrationError(
                'el rectángulo no entra en la imagen (la esquina %s queda afuera): achicá '
                'calib_half_width_m o subí calib_forward_near_m' % nombre)
        warp_src += [round(pixel[0] / intr.width, DECIMALES_WARP),
                     round(pixel[1] / intr.height, DECIMALES_WARP)]
    warp_src = tuple(warp_src)
    if not cuadrilatero_valido(warp_src):
        raise CalibrationError('el trapecio calculado no es un cuadrilátero válido: revisá '
                               'calib_forward_near_m, calib_forward_far_m y calib_half_width_m')
    return warp_src


def calibrar_piso(profundidades: Sequence[np.ndarray], intr: Intrinsics,
                  cfg: GroundCalibrationConfig, vista: GroundView,
                  warp_width: int, warp_height: int) -> GroundCalibration:
    """Mide la pose y calcula la geometría. Lanza CalibrationError si algo no cierra."""
    pose = medir_pose(profundidades, intr, cfg)
    return GroundCalibration(
        pose=pose, intrinsics=intr, warp_src=trapecio(pose, intr, vista),
        mm_per_px_x=1000.0 * vista.width_m / warp_width,
        mm_per_px_y=1000.0 * vista.length_m / warp_height)


# --- de los parámetros del nodo a la configuración del detector ---------------------------

# Parámetros de los que sale la geometría (y no van tal cual a LaneConfig).
PARAMETROS_DE_GEOMETRIA = (
    'calib_height_m', 'calib_pitch_deg', 'calib_roll_deg', 'camera_intrinsics',
    'calib_forward_near_m', 'calib_forward_far_m', 'calib_half_width_m')
# Campos de LaneConfig que se calculan con esos parámetros.
CAMPOS_CALCULADOS = ('warp_src', 'view_width_m', 'view_length_m')


def intrinsecos_de(valores: Sequence[float]) -> Intrinsics:
    """Intrinsics a partir de [fx, fy, cx, cy, ancho, alto]."""
    if len(valores) != 6:
        raise ValueError('camera_intrinsics debe tener 6 valores: fx, fy, cx, cy, ancho, alto')
    fx, fy, cx, cy, ancho, alto = valores
    if min(fx, fy) <= 0.0 or ancho < 2 or alto < 2:
        raise ValueError('camera_intrinsics: fx y fy deben ser positivos y la imagen de al '
                         'menos 2 x 2')
    return Intrinsics(float(fx), float(fy), float(cx), float(cy), int(ancho), int(alto))


def lane_config_desde_parametros(valores: dict):
    """Arma la LaneConfig del detector con los parámetros del nodo (un diccionario).

    `warp_src`, `view_width_m` y `view_length_m` se calculan con la pose, los intrínsecos
    y el rectángulo; todo lo demás pasa tal cual. Lanza ValueError (o CalibrationError) si
    la geometría no es válida.
    """
    pose = GroundPose(valores['calib_height_m'], valores['calib_pitch_deg'],
                      valores['calib_roll_deg'])
    vista = GroundView(valores['calib_forward_near_m'], valores['calib_forward_far_m'],
                       valores['calib_half_width_m'])
    calculados = {
        'warp_src': trapecio(pose, intrinsecos_de(valores['camera_intrinsics']), vista),
        'view_width_m': vista.width_m,
        'view_length_m': vista.length_m,
    }
    directos = {campo.name: valores[campo.name] for campo in dataclasses.fields(LaneConfig)
                if campo.name not in CAMPOS_CALCULADOS}
    for nombre in ('white_hsv_min', 'white_hsv_max', 'width_check_fractions'):
        directos[nombre] = tuple(directos[nombre])
    return LaneConfig(**directos, **calculados)
