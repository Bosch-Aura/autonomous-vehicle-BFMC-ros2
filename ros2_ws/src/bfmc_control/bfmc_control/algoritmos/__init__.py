"""Algoritmos de giro intercambiables (ADR-0004). Se elige uno con el parámetro `algorithm`."""
from bfmc_control.algoritmos.base import AlgoritmoDeGiro
from bfmc_control.algoritmos.pid import Pid
from bfmc_control.algoritmos.pure_pursuit import PurePursuit

# Nombre (el valor de `algorithm`) -> clase. Los nombres son los de tipos.ALGORITMOS.
REGISTRO = {
    'pid': Pid,
    'pure_pursuit': PurePursuit,
}


def crear(nombre: str) -> AlgoritmoDeGiro:
    """Instancia el algoritmo `nombre`. ValueError si no existe."""
    if nombre not in REGISTRO:
        raise ValueError('algoritmo desconocido: %s (hay: %s)' % (nombre, ', '.join(REGISTRO)))
    return REGISTRO[nombre]()
