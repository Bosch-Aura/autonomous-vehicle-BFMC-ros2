"""Corre los 37 casos del análisis (docs/ANALISIS-CARRIL-Y-CONTROL.md) en lazo cerrado.

Uso (desde esta carpeta, con un Python que tenga numpy, opencv y pyyaml):
    python3 casos.py                 # con los params.yaml del repo
    python3 casos.py --antes         # como estaba antes del 2026-11-17 (sin Kalman ni error_shift)
    python3 casos.py --filtro "C der"  # solo los casos cuyo nombre contiene el texto
"""
import argparse
import math
from multiprocessing import Pool

from sim import correr, fmt, Marcas, Seg

P = math.pi
ANTES = dict(kalman=False, ctrl_cambios={'error_shift_m': 0.0, 'filter_alpha': 0.6})


def recta():
    return [Seg('S', 4.0)]


def curva(k, R, ang=P / 2):
    return [Seg('S', 1.2), Seg(k, ang, R), Seg('S', 1.5)]


CASOS = [
    ('R01 recta centrado', recta(), Marcas(), {}),
    ('R02 recta arranca corrido 8cm a la izq', recta(), Marcas(), dict(d0=0.08)),
    ('R03 recta arranca corrido 12cm a la der', recta(), Marcas(), dict(d0=-0.12)),
    ('R04 recta arranca cruzado 20° izq', recta(), Marcas(), dict(yaw0=0.35)),
    ('R05 recta solo ve la izq (der borrada)', recta(), Marcas(right_solid=False), {}),
    ('R06 recta solo ve la der (sin central)', recta(),
     Marcas(center_line='none', adjacent_outer=False), {}),
    ('R07 recta solo der + corrido 10cm der', recta(),
     Marcas(center_line='none', adjacent_outer=False), dict(d0=-0.10)),
    ('R08 recta sin carril vecino', recta(), Marcas(adjacent_outer=False), {}),
    ('R09 recta corrido 10cm izq (vecino cerca)', recta(), Marcas(), dict(d0=0.10, yaw0=0.15)),
    ('R10 recta ciego 0.5 s', recta(), Marcas(blind=[(1.5, 2.0)]), dict(yaw0=0.1)),
    ('R11 recta ciego 1.5 s', recta(), Marcas(blind=[(1.5, 3.0)]), dict(yaw0=0.1)),
    ('R12 recta reflejo junto a la der', recta(), Marcas(blob=(1.2, -0.12, 0.06)), {}),
    ('R13 recta der gastada 40cm', recta(), Marcas(erase=[(1.0, 1.4, 'right')]), {}),
    ('R14 recta linea de pare', recta(), Marcas(stop_line_s=1.6), {}),
    ('R15 recta paso de cebra', recta(), Marcas(crosswalk_s=1.6), {}),
]
# Bifurcaciones (prueba en pista del 2026-11-18): nuestro carril sigue derecho.
BIF = dict(center_line='solid', fork_s=1.3)
CASOS += [
    ('B01 bifurcación a la izq', recta(), Marcas(**BIF), {}),
    ('B02 bifurcación a la izq, abre rápido (R 0.4)', recta(), Marcas(**BIF, fork_R=0.4), {}),
    ('B03 bifurcación a la izq, auto corrido 6cm izq', recta(), Marcas(**BIF), dict(d0=0.06)),
    ('B04 bifurcación a la izq + paso de cebra', recta(), Marcas(**BIF, crosswalk_s=1.5), {}),
    ('B05 bifurcación a la izq sin la línea derecha', recta(),
     Marcas(**BIF, erase=[(0.9, 3.0, 'right')]), {}),
    ('B07 bifurcación a la izq sin la derecha, corrido 5cm izq', recta(),
     Marcas(**BIF, erase=[(0.9, 3.0, 'right')]), dict(d0=0.05)),
    ('B08 bifurcación a la izq sin la derecha, abre rápido', recta(),
     Marcas(**BIF, fork_R=0.4, erase=[(0.9, 3.0, 'right')]), {}),
    ('B06 salida a la der', recta(), Marcas(center_line='solid', fork_s=1.3, fork_side='right'), {}),
]
# Piso blanco fuera de la pista (prueba del 2026-11-18).
CASOS += [
    ('P01 recta, piso blanco a 5cm de la der', recta(), Marcas(piso_der=0.05), {}),
    ('P02 recta, piso blanco a los dos lados', recta(), Marcas(piso_der=0.05, piso_izq=0.05), {}),
    ('P03 recta, piso blanco + línea de pare', recta(), Marcas(piso_der=0.05, stop_line_s=1.6), {}),
    ('P04 curva izq R=0.46, piso blanco afuera', curva('L', 0.46), Marcas(piso_der=0.05), {}),
    ('P05 curva der R=0.66, piso blanco adentro', curva('R', 0.66), Marcas(piso_der=0.05), {}),
]
for k, nom in (('L', 'izq'), ('R', 'der')):
    for R in (1.2, 0.9, 0.66, 0.55, 0.46):
        CASOS.append((f'C {nom} R={R} (con vecino)', curva(k, R), Marcas(), {}))
    for R in (0.9, 0.66):
        CASOS.append((f'C {nom} R={R} sin vecino', curva(k, R), Marcas(adjacent_outer=False), {}))
        CASOS.append((f'C {nom} R={R} ciego 0.5s en la curva', curva(k, R),
                      Marcas(blind=[(5.0, 5.5)]), {}))
        CASOS.append((f'C {nom} R={R} entra corrido 8cm afuera', curva(k, R), Marcas(),
                      dict(d0=(-0.08 if k == 'L' else 0.08))))


def uno(args):
    (nombre, segs, marcas, kw), antes = args
    extra = dict(ANTES) if antes else {}
    return correr(nombre, segs, marcas, **extra, **kw)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--antes', action='store_true')
    ap.add_argument('--filtro', default='')
    a = ap.parse_args()
    elegidos = [c for c in CASOS if a.filtro in c[0]]
    with Pool() as p:
        resultados = p.map(uno, [(c, a.antes) for c in elegidos])
    for r in resultados:
        print(fmt(r))
    curvas = [r for r in resultados if r.nombre.startswith('C ')]
    print(f'\nSin salirse: {sum(not r.salio for r in resultados)}/{len(resultados)} casos'
          f' ({sum(not r.salio for r in curvas)}/{len(curvas)} curvas)')
