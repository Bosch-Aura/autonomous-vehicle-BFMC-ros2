"""Guarda parámetros de un nodo en un archivo YAML propio, fuera del repositorio. Sin ROS.

Por qué fuera del repo: con `colcon build --symlink-install`, el params.yaml instalado es
un enlace al del repo. Escribirlo dejaría cambios sin commitear en la Jetson y conflictos
en cada `git pull`. El archivo guardado se carga en el launch DESPUÉS del params.yaml del
paquete, así sus valores le ganan (ver `archivo_persistido` y los launch).

La escritura es atómica: se escribe un temporal en la misma carpeta y se lo renombra. Si
se corta la luz a mitad de camino, queda el archivo anterior entero, nunca uno a medias.
"""
import os
import tempfile

import yaml

ENCABEZADO = ('# Parámetros guardados por el nodo %s (botón Guardar del visor o su servicio).\n'
              '# Se cargan después de config/params.yaml y le ganan. Para volver a los del\n'
              '# repo, borrá este archivo.\n')


def _plano(valor):
    """Convierte arreglos de ROS (array.array, tuplas) en listas que YAML sabe escribir."""
    if isinstance(valor, (str, bool, int, float)):
        return valor
    return [_plano(v) for v in valor]


def guardar_parametros(ruta: str, nodo: str, valores: dict) -> str:
    """Escribe `valores` como parámetros de `nodo` en `ruta`. Devuelve la ruta absoluta.

    Lanza OSError si no se puede escribir.
    """
    ruta = os.path.abspath(os.path.expanduser(ruta))
    carpeta = os.path.dirname(ruta)
    os.makedirs(carpeta, exist_ok=True)
    contenido = {nodo: {'ros__parameters': {k: _plano(v) for k, v in sorted(valores.items())}}}
    descriptor, temporal = tempfile.mkstemp(dir=carpeta, prefix='.guardando-', suffix='.yaml')
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as archivo:
            archivo.write(ENCABEZADO % nodo)
            yaml.safe_dump(contenido, archivo, allow_unicode=True, sort_keys=False)
            archivo.flush()
            os.fsync(archivo.fileno())
        os.replace(temporal, ruta)
    except BaseException:
        if os.path.exists(temporal):
            os.unlink(temporal)
        raise
    return ruta


def leer_parametros(ruta: str, nodo: str) -> dict:
    """Los parámetros de `nodo` guardados en `ruta`, o {} si el archivo no existe."""
    ruta = os.path.expanduser(ruta)
    if not os.path.exists(ruta):
        return {}
    with open(ruta, encoding='utf-8') as archivo:
        contenido = yaml.safe_load(archivo) or {}
    return contenido.get(nodo, {}).get('ros__parameters', {})


def archivo_persistido(params_yaml: str, nodo: str):
    """La ruta del archivo guardado de `nodo` si existe, o None. Para los launch.

    La ruta sale del parámetro `persist_file` del params.yaml del paquete.
    """
    with open(params_yaml, encoding='utf-8') as archivo:
        parametros = yaml.safe_load(archivo)[nodo]['ros__parameters']
    ruta = parametros.get('persist_file')
    if not ruta:
        return None
    ruta = os.path.expanduser(ruta)
    return ruta if os.path.exists(ruta) else None
