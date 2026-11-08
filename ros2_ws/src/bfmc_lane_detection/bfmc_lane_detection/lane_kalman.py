"""Filtro de Kalman del estado del carril (A2-154). Lógica pura: sin ROS ni reloj.

Estado x = [d, psi, kappa], en la fila de referencia del detector (dentro de la vista
cenital: medir en el auto obligaría a extrapolar el polinomio fuera de los datos):

- d: desvío lateral [m], positivo si el centro del carril está a la izquierda;
- psi: rumbo del carril respecto del auto [rad], positivo hacia la izquierda;
- kappa: curvatura del carril [1/m], positiva hacia la izquierda.

Modelo de proceso, ángulo chico, con el avance del auto s = v·dt:

    d' = d + s·psi        psi' = psi + s·kappa        kappa' = kappa

No se resta el giro propio del auto (necesitaría la distancia entre ejes y el ángulo de
dirección): lo absorbe el ruido de proceso Q. Si en la pista se ve atraso en las curvas,
es la primera mejora a probar.

La medición es el estado completo (H = I). Q está por segundo, así el filtro se comporta
igual a cualquier cantidad de cuadros por segundo. Las mediciones demasiado lejos de la
predicción (distancia de Mahalanobis al cuadrado mayor a `gate_chi2`; 11,34 es el 99 %
de una chi-cuadrado con 3 grados de libertad) se descartan: es un reflejo o un salto.
"""
from typing import Optional, Sequence

import numpy as np


def _diagonal(valores: Sequence[float], nombre: str) -> np.ndarray:
    if len(valores) != 3 or any(v <= 0.0 for v in valores):
        raise ValueError('%s debe tener 3 varianzas positivas: desvío, rumbo y curvatura'
                         % nombre)
    return np.diag(np.asarray(valores, dtype=np.float64))


class LaneKalman:
    """Estima [desvío, rumbo, curvatura] con predicción por el avance y corrección."""

    def __init__(self, q: Sequence[float], r_two_lines: Sequence[float],
                 r_one_line: Sequence[float], gate_chi2: float):
        self._q = _diagonal(q, 'kalman_q')
        self._r = {True: _diagonal(r_two_lines, 'kalman_r_two_lines'),
                   False: _diagonal(r_one_line, 'kalman_r_one_line')}
        if gate_chi2 <= 0.0:
            raise ValueError('kalman_gate_chi2 debe ser positivo')
        self._gate = gate_chi2
        self._x: Optional[np.ndarray] = None
        self._p: Optional[np.ndarray] = None

    @property
    def inicializado(self) -> bool:
        return self._x is not None

    @property
    def estado(self) -> tuple:
        """(d, psi, kappa). Solo vale si `inicializado`."""
        return tuple(float(v) for v in self._x)

    @property
    def desvio_std_m(self) -> float:
        """Desviación estándar del desvío lateral [m]: qué tan seguro está el filtro."""
        return float(np.sqrt(self._p[0, 0]))

    def reiniciar(self, medicion: Sequence[float], dos_lineas: bool) -> None:
        """Arranca de cero con la medición: el estado es ella y la incertidumbre, su R."""
        self._x = np.asarray(medicion, dtype=np.float64).copy()
        self._p = self._r[dos_lineas].copy()

    def olvidar(self) -> None:
        self._x = None
        self._p = None

    def predecir(self, dt: float, velocidad_mps: float) -> None:
        """Avanza el estado `dt` segundos. Sin efecto si no está inicializado o dt <= 0."""
        if not self.inicializado or dt <= 0.0:
            return
        avance = velocidad_mps * dt
        f = np.array([[1.0, avance, 0.0], [0.0, 1.0, avance], [0.0, 0.0, 1.0]])
        self._x = f @ self._x
        self._p = f @ self._p @ f.T + self._q * dt

    def corregir(self, medicion: Sequence[float], dos_lineas: bool) -> bool:
        """Corrige con la medición. Devuelve False si se descartó por estar demasiado lejos.

        `dos_lineas` elige R: la medición con una sola línea pesa menos.
        """
        z = np.asarray(medicion, dtype=np.float64)
        innovacion = z - self._x
        s = self._p + self._r[dos_lineas]
        if float(innovacion @ np.linalg.solve(s, innovacion)) > self._gate:
            return False
        ganancia = self._p @ np.linalg.inv(s)
        self._x = self._x + ganancia @ innovacion
        p = (np.eye(3) - ganancia) @ self._p
        self._p = 0.5 * (p + p.T)    # simétrica, contra el error de redondeo acumulado
        return True
