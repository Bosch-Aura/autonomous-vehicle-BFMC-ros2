"""Estado de giro: el control se acuerda de que está en una curva (ADR-0004, prueba 1).

Con una sola línea el detector a veces cambia de lado o mide un rumbo al revés, y el control
obedecía cada medición: en el video de la prueba 1 el giro objetivo pasó de +25° a -16,5° y
otra vez a +25° en menos de un segundo, con el carril doblando a la izquierda todo el tiempo.

Esta clase entra a "curva a la izquierda" (o a la derecha) cuando el rumbo del carril se
mantiene por encima de `curve_enter_heading_rad` durante `curve_enter_s`. Mientras dura:

- un giro objetivo de signo contrario al de la curva se reemplaza por el último buen giro,
  hasta `curve_hold_s`, y después por 0 (nunca se gira al revés por una medición suelta);
- si se pierde el carril se sostiene el giro `curve_lost_hold_s`.

Sale cuando el rumbo queda por debajo de `curve_exit_heading_rad` durante `curve_exit_s`, o a
los `curve_max_s`. Si el rumbo se mantiene del lado contrario `curve_reverse_s`, es una curva en
S: pasa a la dirección nueva. Con `curve_enter_heading_rad` en 0 está apagado.

Es lógica pura: recibe el instante actual como argumento. Convención: positivo a la izquierda.
"""
import math


def _signo(valor: float) -> int:
    return 1 if valor > 0.0 else -1 if valor < 0.0 else 0


class EstadoDeGiro:
    """Recuerda si el auto está en una curva y de qué lado."""

    def __init__(self):
        self.reiniciar()

    def reiniciar(self) -> None:
        """Sale de la curva y olvida todo."""
        self._direccion = 0              # +1 izquierda, -1 derecha, 0 sin curva
        self._inicio = None              # instante en que entró
        self._candidato = 0              # lado hacia el que el rumbo pasa del umbral
        self._candidato_desde = None
        self._recto_desde = None         # desde cuándo el rumbo está bajo el umbral de salida
        self._buen_giro = 0.0            # último giro objetivo del lado de la curva
        self._buen_giro_instante = None

    @property
    def direccion(self) -> int:
        """+1 en una curva a la izquierda, -1 a la derecha, 0 sin curva."""
        return self._direccion

    def activo(self, cfg, now: float) -> bool:
        """Si está en una curva. Vence solo a los `curve_max_s`."""
        if self._direccion != 0 and now - self._inicio > cfg.curve_max_s:
            self._salir()
        return self._direccion != 0

    def actualizar(self, cfg, rumbo: float, now: float) -> None:
        """Toma el rumbo del carril usable de este ciclo."""
        if cfg.curve_enter_heading_rad <= 0.0:
            self.reiniciar()
            return
        if not math.isfinite(rumbo):
            return
        self.activo(cfg, now)
        self._actualizar_candidato(cfg, rumbo, now)
        if self._direccion == 0:
            if self._sostenido(self._candidato_desde, cfg.curve_enter_s, now):
                self._entrar(self._candidato, now)
            return
        if (self._candidato == -self._direccion
                and self._sostenido(self._candidato_desde, cfg.curve_reverse_s, now)):
            self._entrar(self._candidato, now)
            return
        self._actualizar_salida(cfg, rumbo, now)

    def corregir(self, cfg, objetivo: float, now: float) -> float:
        """El giro objetivo que se puede usar: el mismo, o el último bueno si va al revés."""
        if not self.activo(cfg, now):
            return objetivo
        if objetivo * self._direccion >= 0.0:
            self._buen_giro = objetivo
            self._buen_giro_instante = now
            return objetivo
        if (self._buen_giro_instante is not None
                and now - self._buen_giro_instante <= cfg.curve_hold_s):
            return self._buen_giro
        return 0.0

    def sostener_s(self, cfg, now: float) -> float:
        """Cuánto sostener el último giro al perder el carril."""
        if self.activo(cfg, now):
            return max(cfg.hold_steer_s, cfg.curve_lost_hold_s)
        return cfg.hold_steer_s

    # --- internos --------------------------------------------------------------------

    def _actualizar_candidato(self, cfg, rumbo, now):
        lado = _signo(rumbo) if abs(rumbo) >= cfg.curve_enter_heading_rad else 0
        if lado != self._candidato:
            self._candidato = lado
            self._candidato_desde = now if lado != 0 else None

    @staticmethod
    def _sostenido(desde, duracion, now) -> bool:
        return desde is not None and now - desde >= duracion

    def _entrar(self, direccion, now):
        self._direccion = direccion
        self._inicio = now
        self._recto_desde = None
        self._buen_giro = 0.0
        self._buen_giro_instante = None

    def _salir(self):
        self._direccion = 0
        self._inicio = None
        self._recto_desde = None
        self._buen_giro_instante = None

    def _actualizar_salida(self, cfg, rumbo, now):
        if abs(rumbo) >= cfg.curve_exit_heading_rad:
            self._recto_desde = None
            return
        if self._recto_desde is None:
            self._recto_desde = now
        elif now - self._recto_desde >= cfg.curve_exit_s:
            self._salir()
