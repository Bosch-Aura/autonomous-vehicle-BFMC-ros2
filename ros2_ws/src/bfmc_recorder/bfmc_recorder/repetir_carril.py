"""Repite una grabación de bfmc_recorder por el detector de carril con otros parámetros.

No necesita ROS corriendo ni el auto: lee `frames/*.jpg` y `params_lane_detection.yaml` de la
carpeta de la grabación y pasa cada cuadro por el mismo detector y el mismo tracker que usa el
nodo `lane_detection`. Hace falta tener instalado (o en PYTHONPATH) `bfmc_lane_detection`.

    python3 -m bfmc_recorder.repetir_carril GRABACION
    python3 -m bfmc_recorder.repetir_carril GRABACION detector=l_adaptativo l_offset=20
    python3 -m bfmc_recorder.repetir_carril GRABACION --grilla calib_half_width_m=0.30,0.36 \
        lane_width_tolerance_m=0.05,0.08,0.11 detector=l_adaptativo

Cada argumento `nombre=valor` pisa un parámetro del detector. Con `--grilla`, los que tienen
varios valores separados por coma se combinan todos con todos, en paralelo, y se imprime una
fila de métricas por combinación (ver `metricas`).
"""
import glob
import itertools
import math
import multiprocessing
import os
import statistics
import sys

import cv2
import yaml


def _valor(texto):
    """'0.3' -> 0.3, '5' -> 5, 'true' -> True, 'l_adaptativo' -> 'l_adaptativo'."""
    try:
        return yaml.safe_load(texto)
    except yaml.YAMLError:
        return texto


def repetir(carpeta, cambios=None):
    """Pasa los cuadros de la grabación por el detector. Devuelve una fila por cuadro."""
    from bfmc_lane_detection import detectores
    from bfmc_lane_detection.ground_calibration import lane_config_desde_parametros
    from bfmc_lane_detection.lane_tracker import LaneTracker

    with open(os.path.join(carpeta, 'params_lane_detection.yaml'), encoding='utf-8') as archivo:
        parametros = dict(next(iter(yaml.safe_load(archivo).values()))['ros__parameters'])
    parametros.update(cambios or {})
    detector = detectores.crear(lane_config_desde_parametros(parametros))
    tracker = LaneTracker(parametros['lost_timeout_s'])
    tracker.memoria_lado_s = parametros['side_memory_s']
    archivos = sorted(glob.glob(os.path.join(carpeta, 'frames', '*.jpg')),
                      key=lambda ruta: int(os.path.basename(ruta)[:-4]))
    inicio = int(os.path.basename(archivos[0])[:-4])
    filas = []
    for ruta in archivos:
        instante_ns = int(os.path.basename(ruta)[:-4])
        instante = instante_ns * 1e-9
        medicion = detector.detect(cv2.imread(ruta), tracker.previas(instante),
                                   tracker.centro_previo(instante))
        estado = tracker.update(medicion, instante, 0.0)
        filas.append({
            't': (instante_ns - inicio) * 1e-9, 'modo': estado.mode,
            'lateral': estado.lateral_error_m, 'rumbo': estado.heading_error_rad,
            'confianza': estado.confidence, 'izq': medicion.left_valid,
            'der': medicion.right_valid, 'fallo': medicion.failed_checks})
    return filas


def metricas(filas):
    """Porcentaje de cuadros por modo, cambios de lado, saltos de rumbo y desvío medio."""
    total = len(filas)
    por_modo = [100.0 * sum(1 for f in filas if f['modo'] == m) / total for m in range(4)]
    lado, cambios, saltos, anterior = None, 0, 0, None
    for fila in filas:
        if fila['modo'] == 1:
            actual = 'izq' if fila['izq'] else 'der'
            cambios += lado is not None and actual != lado
            lado = actual
            saltos += anterior is not None and abs(fila['rumbo'] - anterior) > 0.4
        anterior = fila['rumbo'] if fila['modo'] in (0, 1) else None
    medidos = [f for f in filas if math.isfinite(f['lateral'])]
    return {'cuadros': total, 'dos_lineas_%': por_modo[0], 'una_linea_%': por_modo[1],
            'predicho_%': por_modo[2], 'perdido_%': por_modo[3], 'cambios_de_lado': cambios,
            'saltos_de_rumbo': saltos,
            'desvio_medio_cm': 100.0 * statistics.mean(abs(f['lateral']) for f in medidos)
            if medidos else math.nan}


def _evaluar(argumentos):
    carpeta, cambios = argumentos
    try:
        return cambios, metricas(repetir(carpeta, cambios))
    except (ValueError, KeyError) as error:
        return cambios, {'error': str(error)[:80]}


def _imprimir(resultados):
    print('%-70s %6s %6s %6s %6s %7s %7s %8s' % (
        'cambios', 'dos%', 'una%', 'pred%', 'perd%', 'lados', 'saltos', 'desv_cm'))
    for cambios, m in resultados:
        texto = ' '.join('%s=%s' % par for par in cambios.items()) or '(los grabados)'
        if 'error' in m:
            print('%-70s ERROR: %s' % (texto, m['error']))
            continue
        print('%-70s %6.1f %6.1f %6.1f %6.1f %7d %7d %8.2f' % (
            texto, m['dos_lineas_%'], m['una_linea_%'], m['predicho_%'], m['perdido_%'],
            m['cambios_de_lado'], m['saltos_de_rumbo'], m['desvio_medio_cm']))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 0
    grilla = '--grilla' in argv
    argv = [a for a in argv if a != '--grilla']
    carpeta, pares = argv[0], [a.split('=', 1) for a in argv[1:]]
    opciones = [[(nombre, _valor(v)) for v in valores.split(',')] if grilla
                else [(nombre, _valor(valores))] for nombre, valores in pares]
    combinaciones = [dict(c) for c in itertools.product(*opciones)]
    with multiprocessing.Pool() as pool:
        _imprimir(pool.map(_evaluar, [(carpeta, c) for c in combinaciones]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
