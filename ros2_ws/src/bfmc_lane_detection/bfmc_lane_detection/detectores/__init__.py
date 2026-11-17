"""Detectores de carril intercambiables (ADR-0004). Se elige uno con el parámetro `detector`.

Todos comparten el pipeline de lane_detection_logic.LaneDetector (vista cenital, ventanas,
polinomio, chequeos, medición) y cambian solo la segmentación. Para agregar uno: una clase
que herede de LaneDetector y reemplace `_segmentar`, en un archivo de esta carpeta, más una
línea en REGISTRO.
"""
from bfmc_lane_detection.detectores.hsv_ventanas import HsvVentanas
from bfmc_lane_detection.detectores.l_adaptativo import LAdaptativo
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector

# Nombre (el valor de `detector`) -> clase.
REGISTRO = {
    'hsv_ventanas': HsvVentanas,
    'l_adaptativo': LAdaptativo,
}


def crear(config: LaneConfig) -> LaneDetector:
    """Instancia el detector que pide `config.detector`. ValueError si no existe."""
    if config.detector not in REGISTRO:
        raise ValueError('detector desconocido: %s (hay: %s)' % (
            config.detector, ', '.join(REGISTRO)))
    return REGISTRO[config.detector](config)
