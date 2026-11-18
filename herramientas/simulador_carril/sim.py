"""Simulador de lazo cerrado: pista sintética -> detector real -> tracker real -> control real.

Dibuja la vista cenital (0,80 x 0,60 m, 640 x 400 px, como params.yaml) desde la pose del auto
en una pista hecha de rectas y arcos, la pasa por LaneDetector + LaneTracker + ControlLogic del
repo con los parámetros de params.yaml, y mueve el auto con un modelo de bicicleta.
Lo que NO modela: la cámara real (perspectiva, desenfoque, luz), el servo, el motor, el
deslizamiento. Sirve para ver la LÓGICA, no para afinar números finos.
"""
import math
import os
import sys
from dataclasses import dataclass, field, replace

import numpy as np
import yaml

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ros2_ws', 'src')
sys.path[:0] = [os.path.join(REPO, 'bfmc_lane_detection'), os.path.join(REPO, 'bfmc_control')]

from bfmc_lane_detection.lane_detection_logic import LaneConfig  # noqa: E402
from bfmc_lane_detection import detectores  # noqa: E402
from bfmc_lane_detection.lane_kalman import LaneKalman  # noqa: E402
from bfmc_lane_detection.lane_tracker import LaneTracker, MODE_LOST, MODE_PREDICTED, MODE_ONE_LINE, MODE_TWO_LINES  # noqa
from bfmc_control.control_logic import ControlLogic, ControlConfig, LaneInput  # noqa: E402
from bfmc_control.pure_pursuit import PurePursuitConfig  # noqa: E402

LP = yaml.safe_load(open(os.path.join(REPO, 'bfmc_lane_detection/config/params.yaml')))['lane_detection']['ros__parameters']
CP = yaml.safe_load(open(os.path.join(REPO, 'bfmc_control/config/params.yaml')))['control']['ros__parameters']

if os.environ.get('FAR'): LP['calib_forward_far_m'] = float(os.environ['FAR'])
if os.environ.get('HALF'):
    LP['calib_half_width_m'] = float(os.environ['HALF']); LP['warp_width'] = int(round(LP['calib_half_width_m'] * 2 / 0.00125))
if os.environ.get('FAR'): LP['warp_height'] = int(round((LP['calib_forward_far_m'] - LP['calib_forward_near_m']) / 0.0015))
W_PX, H_PX = LP['warp_width'], LP['warp_height']
VIEW_W = 2 * LP['calib_half_width_m']
VIEW_L = LP['calib_forward_far_m'] - LP['calib_forward_near_m']
MPP_X, MPP_Y = VIEW_W / W_PX, VIEW_L / H_PX
CAM_TO_AXLE = CP['camera_to_rear_axle_m']
NEAR = LP['calib_forward_near_m']
LANE_W = LP['lane_width_m']

# Coordenadas de cada píxel de la vista cenital en el marco del auto (eje trasero, x adelante, y izq).
_u, _v = np.meshgrid(np.arange(W_PX) + 0.5, np.arange(H_PX) + 0.5)
PX_X = CAM_TO_AXLE + NEAR + (H_PX - _v) * MPP_Y
PX_Y = (W_PX / 2.0 - _u) * MPP_X


def lane_config(**cambios):
    nombres = [f for f in LaneConfig.__dataclass_fields__]
    base = {k: LP[k] for k in nombres if k in LP}
    base.update(dict(warp_src=(0, 0, 1, 0, 1, 1, 0, 1), view_width_m=VIEW_W, view_length_m=VIEW_L,
                     roi_top_fraction=0.0))
    base.update(cambios)
    return LaneConfig(**base)


def control_config(**cambios):
    pp_campos = PurePursuitConfig.__dataclass_fields__
    pp = PurePursuitConfig(**{k: CP[k] for k in pp_campos if k in CP})
    campos = ControlConfig.__dataclass_fields__
    base = {k: CP[k] for k in campos if k in CP and k != 'pure_pursuit'}
    base['pure_pursuit'] = pp
    base.update(cambios)
    return ControlConfig(**base)


# ---------------------------------------------------------------- pista
@dataclass
class Seg:
    kind: str          # 'S' recta, 'L' arco a la izquierda, 'R' arco a la derecha
    length: float      # recta: largo; arco: ángulo en rad
    radius: float = 0.0


class Track:
    """Línea central de NUESTRO carril como rectas y arcos. d > 0 = a la izquierda."""

    def __init__(self, segs):
        self.parts = []
        x, y, th, s = 0.0, 0.0, 0.0, 0.0
        for g in segs:
            if g.kind == 'S':
                self.parts.append(('S', x, y, th, s, g.length))
                x += g.length * math.cos(th); y += g.length * math.sin(th); s += g.length
            else:
                sg = 1.0 if g.kind == 'L' else -1.0
                cx, cy = x - sg * g.radius * math.sin(th), y + sg * g.radius * math.cos(th)
                self.parts.append((g.kind, cx, cy, th, s, g.length, g.radius))
                th2 = th + sg * g.length
                x, y = cx + sg * g.radius * math.sin(th2), cy - sg * g.radius * math.cos(th2)
                th = th2; s += g.radius * g.length
        self.total = s

    def sd(self, X, Y):
        """(s, d) de puntos del mundo respecto de la línea central (la parte más cercana)."""
        best_d = np.full(np.shape(X), np.inf); best_s = np.zeros(np.shape(X))
        for p in self.parts:
            if p[0] == 'S':
                _, x0, y0, th, s0, ln = p
                dx, dy = X - x0, Y - y0
                a = dx * math.cos(th) + dy * math.sin(th)
                d = -dx * math.sin(th) + dy * math.cos(th)
                ok = (a >= -0.3 if s0 == 0 else a >= 0) & (a <= ln + (0.6 if p is self.parts[-1] else 0))
                s = s0 + a
            else:
                kind, cx, cy, th, s0, ang, R = p
                sg = 1.0 if kind == 'L' else -1.0
                dx, dy = X - cx, Y - cy
                r = np.hypot(dx, dy)
                phi = np.arctan2(dy, dx)
                phi0 = th - sg * math.pi / 2
                rel = sg * np.angle(np.exp(1j * (phi - phi0)))
                rel = np.where(rel < -0.5, rel + 2 * math.pi, rel)
                ok = (rel >= 0) & (rel <= ang)
                d = sg * (R - r)
                s = s0 + R * rel
            mejor = ok & (np.abs(d) < np.abs(best_d))
            best_d = np.where(mejor, d, best_d); best_s = np.where(mejor, s, best_s)
        return best_s, best_d

    def pose_at(self, s_target, d=0.0):
        """Pose (x, y, th) sobre la línea central a la distancia s (más d a la izquierda)."""
        for p in self.parts:
            if p[0] == 'S':
                _, x0, y0, th, s0, ln = p
                if s_target <= s0 + ln:
                    a = s_target - s0
                    return (x0 + a * math.cos(th) - d * math.sin(th), y0 + a * math.sin(th) + d * math.cos(th), th)
            else:
                kind, cx, cy, th, s0, ang, R = p
                if s_target <= s0 + R * ang:
                    sg = 1.0 if kind == 'L' else -1.0
                    th2 = th + sg * (s_target - s0) / R
                    rr = R - sg * d
                    return (cx + sg * rr * math.sin(th2), cy - sg * rr * math.cos(th2), th2)
        raise ValueError('fuera de la pista')

    def curv_at(self, s):
        for p in self.parts:
            ln = p[5] if p[0] == 'S' else p[6] * p[5]
            if s <= p[4] + ln:
                return 0.0 if p[0] == 'S' else (1 if p[0] == 'L' else -1) / p[6]
        return 0.0


# ---------------------------------------------------------------- dibujo
@dataclass
class Marcas:
    """Qué hay pintado. Offsets respecto de la línea central de nuestro carril."""
    thickness: float = 0.02                 # [asumido] ancho de la línea pintada
    right_solid: bool = True                # borde derecho de nuestro carril (d = -w/2)
    center_line: str = 'dashed'             # 'solid' | 'dashed' | 'none' (d = +w/2)
    dash: float = 0.10                      # [asumido] trazo / hueco de la punteada
    gap: float = 0.10
    adjacent_outer: bool = True             # borde izquierdo del carril de al lado (d = +3w/2)
    stop_line_s: float = None               # línea de pare transversal en nuestro carril
    crosswalk_s: float = None               # paso de cebra (franjas paralelas al carril)
    blob: tuple = None                      # (s, d, radio) mancha de reflejo
    erase: list = field(default_factory=list)  # [(s0, s1, 'right'|'center'|'all')] tramos borrados
    blind: list = field(default_factory=list)  # [(t0, t1)] cuadros sin nada (encandilamiento)
    # Bifurcación: desde fork_s, una línea se abre en «Y» con un arco tangente de radio fork_R.
    # 'left': el carril de al lado se separa a la izquierda (su línea derecha sale de nuestra
    # línea izquierda, que sigue derecho). 'right': una salida a la derecha (una rama sale de
    # nuestra línea derecha, que sigue derecho). Nuestro carril siempre sigue derecho.
    fork_s: float = None
    fork_R: float = 0.6
    fork_side: str = 'left'


def render(track, marcas, pose, t):
    X0, Y0, th = pose
    c, s_ = math.cos(th), math.sin(th)
    X = X0 + PX_X * c - PX_Y * s_
    Y = Y0 + PX_X * s_ + PX_Y * c
    S, D = track.sd(X, Y)
    h = marcas.thickness / 2
    w = LANE_W
    m = np.zeros(S.shape, bool)
    for t0, t1 in marcas.blind:
        if t0 <= t < t1:
            return np.zeros(S.shape + (3,), np.uint8)

    def borrado(tipo):
        out = np.zeros(S.shape, bool)
        for s0, s1, k in marcas.erase:
            if k in (tipo, 'all'):
                out |= (S >= s0) & (S <= s1)
        return out
    if marcas.right_solid:
        m |= (np.abs(D + w / 2) < h) & ~borrado('right')
    if marcas.center_line != 'none':
        cl = np.abs(D - w / 2) < h
        if marcas.center_line == 'dashed':
            cl &= np.mod(S, marcas.dash + marcas.gap) < marcas.dash
        m |= cl & ~borrado('center')
    if marcas.adjacent_outer:
        m |= (np.abs(D - 1.5 * w) < h) & ~borrado('adjacent')
    if marcas.stop_line_s is not None:
        m |= (np.abs(S - marcas.stop_line_s) < 0.025) & (D > -w / 2) & (D < w / 2)
    if marcas.crosswalk_s is not None:
        en = (S > marcas.crosswalk_s) & (S < marcas.crosswalk_s + 0.30) & (D > -w / 2) & (D < 1.5 * w)
        m |= en & (np.mod(D + w / 2, 0.10) < 0.05)
    if marcas.fork_s is not None:
        u = S - marcas.fork_s
        R = marcas.fork_R
        en = (u > 0) & (u < 0.95 * R)
        abre = np.where(en, R - np.sqrt(np.maximum(R * R - u * u, 0.0)), 0.0)
        if marcas.fork_side == 'left':
            m |= en & (np.abs(D - (w / 2 + abre)) < h)
            if marcas.adjacent_outer:
                m &= ~(en & (np.abs(D - 1.5 * w) < h))          # el borde vecino también se va
                m |= en & (np.abs(D - (1.5 * w + abre)) < h)
        else:
            m |= en & (np.abs(D - (-w / 2 - abre)) < h)
    if marcas.blob is not None:
        bs, bd, br = marcas.blob
        m |= (S - bs) ** 2 + (D - bd) ** 2 < br ** 2
    img = np.zeros(S.shape + (3,), np.uint8)
    img[m] = 255
    return img


# ---------------------------------------------------------------- lazo cerrado
@dataclass
class Resultado:
    nombre: str
    max_abs_d: float = 0.0
    salio: bool = False
    t_salida: float = None
    frames: int = 0
    modos: dict = field(default_factory=lambda: {0: 0, 1: 0, 2: 0, 3: 0})
    lost_max_s: float = 0.0
    lost_primero: float = None
    inter_streak_max: int = 0
    inter_primero: float = None
    lado_errado: int = 0                    # cuadros con el centro medido del lado equivocado
    lado_errado_t: list = field(default_factory=list)
    giro_contrario: int = 0                 # cuadros girando al revés de la curva en una curva
    completo: bool = False
    s_final: float = 0.0
    errores: list = field(default_factory=list)


def correr(nombre, segs, marcas=None, s0=0.2, d0=0.0, yaw0=0.0, v=None, latency=0.1,
           lane_cambios=None, ctrl_cambios=None, fps=30.0, detector='hsv_ventanas',
           t_max=None, salida_m=None, kalman=None):
    marcas = marcas or Marcas()
    track = Track(segs)
    det = detectores.crear(lane_config(detector=detector, **(lane_cambios or {})))
    tracker = LaneTracker(LP['lost_timeout_s'])
    tracker.memoria_lado_s = LP['side_memory_s']
    kalman = LP['kalman_enabled'] if kalman is None else kalman
    if kalman:
        x_ref = LP['kalman_camera_to_rear_axle_m'] + NEAR + LP['reference_row_fraction'] * VIEW_L
        tracker.usar_kalman(LaneKalman(LP['kalman_q'], LP['kalman_r_two_lines'], LP['kalman_r_one_line'],
                                       LP['kalman_gate_chi2'], wheelbase_m=LP['kalman_wheelbase_m'],
                                       x_ref_m=x_ref), LP['kalman_max_std_m'])
    cc = control_config(**(ctrl_cambios or {}))
    ctrl = ControlLogic(cc)
    L = cc.pure_pursuit.wheelbase_m
    v = cc.cruise_speed_mps if v is None else v
    x, y, th = track.pose_at(s0, d0)
    th += yaw0
    dt = 1.0 / fps
    cola = []                       # comandos pendientes por la latencia
    steer = 0.0
    ultimo_cmd = 0.0
    res = Resultado(nombre)
    t = 0.0
    lost_desde = None
    streak = 0
    salida_m = LANE_W / 2 if salida_m is None else salida_m
    t_max = t_max or (track.total - s0) / max(v, 0.05) + 3.0
    while t < t_max:
        S, D = track.sd(np.array([x]), np.array([y]))
        s_car, d_car = float(S[0]), float(D[0])
        if not math.isfinite(d_car) or s_car > track.total - 0.7:
            res.completo = math.isfinite(d_car)
            break
        res.max_abs_d = max(res.max_abs_d, abs(d_car))
        if abs(d_car) > salida_m and not res.salio:
            res.salio, res.t_salida = True, round(t, 2)
        if abs(d_car) > LANE_W * 1.5:
            break
        img = render(track, marcas, (x, y, th), t)
        try:
            r = det.detect(img, tracker.previas(t), tracker.centro_previo(t))
        except Exception as e:  # noqa
            res.errores.append(repr(e)); break
        tl = tracker.update(r, t, v, ultimo_cmd)
        res.frames += 1
        res.modos[tl.mode] += 1
        if tl.mode == MODE_LOST:
            lost_desde = t if lost_desde is None else lost_desde
            res.lost_max_s = max(res.lost_max_s, t - lost_desde + dt)
            if res.lost_primero is None:
                res.lost_primero = round(t, 2)
        else:
            lost_desde = None
        streak = streak + 1 if tl.intersection_detected else 0
        if streak > res.inter_streak_max:
            res.inter_streak_max = streak
        if streak >= 3 and res.inter_primero is None:
            res.inter_primero = round(t, 2)
        # Verdad: desvío del centro del carril en la fila de medición, visto desde el auto.
        xr = CAM_TO_AXLE + NEAR + LP['reference_row_fraction'] * VIEW_L
        Sr, Dr = track.sd(np.array([x + xr * math.cos(th)]), np.array([y + xr * math.sin(th)]))
        verdad = -float(Dr[0])
        if tl.detected and tl.mode in (MODE_ONE_LINE, MODE_TWO_LINES) and math.isfinite(verdad):
            if abs(tl.lateral_error_m - verdad) > 0.6 * LANE_W:
                res.lado_errado += 1
                if len(res.lado_errado_t) < 6:
                    res.lado_errado_t.append((round(t, 2), round(tl.lateral_error_m, 3), round(verdad, 3), tl.mode))
        out = ctrl.update(LaneInput(tl.detected, tl.lateral_error_m, tl.heading_error_rad,
                                    tl.confidence, tl.curvature, xr - CAM_TO_AXLE), t)
        kc = track.curv_at(s_car + 0.3)
        if kc != 0 and out.steering_rad * kc < -0.05:
            res.giro_contrario += 1
        ultimo_cmd = out.steering_rad
        cola.append((t + latency, out.steering_rad, out.speed_mps))
        while cola and cola[0][0] <= t + 1e-9:
            _, steer, v_cmd = cola.pop(0)
        # Bicicleta (eje trasero).
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v / L * math.tan(steer) * dt
        t += dt
    res.s_final = round(s_car, 2)
    return res


def fmt(r):
    tot = max(1, r.frames)
    pm = {k: round(100 * n / tot) for k, n in r.modos.items()}
    out = (f"{r.nombre:<46} {'SALE' if r.salio else 'ok  '} max|d|={r.max_abs_d:.3f}"
           f" modos%[2L,1L,PRED,LOST]={[pm[0], pm[1], pm[2], pm[3]]}"
           f" LOST={r.lost_max_s:.2f}s inter_streak={r.inter_streak_max}"
           f" lado_errado={r.lado_errado} giro_contra={r.giro_contrario}")
    if r.t_salida is not None:
        out += f" t_sale={r.t_salida}"
    if r.lost_primero is not None:
        out += f" t_lost={r.lost_primero}"
    if r.inter_primero is not None:
        out += f" t_inter={r.inter_primero}"
    if r.lado_errado_t:
        out += f" ej_lado={r.lado_errado_t[:3]}"
    if r.errores:
        out += f" ERR={r.errores}"
    if not r.completo and not r.errores:
        out += f" (no terminó, s={r.s_final})"
    return out
