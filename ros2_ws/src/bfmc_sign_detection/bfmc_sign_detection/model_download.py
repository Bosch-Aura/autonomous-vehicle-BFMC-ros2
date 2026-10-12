"""Descarga del modelo de señales desde un release de GitHub, verificando su checksum.

El modelo pesa 52 MB: no vive en el repositorio, porque quedaría para siempre en el
historial de git. Se publica como asset de un release y se baja con este script.

El asset se resuelve por la API de GitHub (por tag y nombre) y no por la URL
pública de descarga, y se baja a un archivo temporal que solo se renombra si el
checksum SHA-256 coincide: un archivo a medias o corrupto nunca queda en su lugar.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import urllib.request

import yaml

API = 'https://api.github.com/repos/%s/releases/tags/%s'
TAMANO_DE_BLOQUE = 1 << 20
PATRON_SHA256 = re.compile(r'^[0-9a-f]{64}$')


class ModelDownloadError(RuntimeError):
    """No se pudo obtener un modelo válido."""


@dataclass(frozen=True)
class ModelRelease:
    """Dónde está publicado el modelo y cuál es su checksum."""

    repository: str
    tag: str
    asset: str
    sha256: str

    def __post_init__(self):
        if self.repository.count('/') != 1:
            raise ValueError('repository debe tener la forma "organización/repositorio"')
        if not self.tag or not self.asset:
            raise ValueError('tag y asset no pueden estar vacíos')
        if not PATRON_SHA256.match(self.sha256):
            raise ValueError('sha256 debe tener 64 caracteres hexadecimales en minúscula')


def cargar_release(ruta) -> ModelRelease:
    """Lee la descripción del release desde config/model.yaml."""
    with open(ruta, encoding='utf-8') as archivo:
        datos = yaml.safe_load(archivo)
    return ModelRelease(
        repository=datos['repository'], tag=datos['tag'], asset=datos['asset'],
        sha256=datos['sha256'])


def sha256_de(ruta) -> str:
    resumen = hashlib.sha256()
    with open(ruta, 'rb') as archivo:
        for bloque in iter(lambda: archivo.read(TAMANO_DE_BLOQUE), b''):
            resumen.update(bloque)
    return resumen.hexdigest()


def _abrir(url, abrir, aceptar):
    pedido = urllib.request.Request(url, headers={
        'Accept': aceptar, 'User-Agent': 'bfmc-sign-detection'})
    return abrir(pedido)


def descargar_modelo(release: ModelRelease, destino, abrir=urllib.request.urlopen,
                     informar=print) -> Path:
    """Deja el modelo en `destino` con el checksum verificado y devuelve su ruta.

    Si `destino` ya existe y su checksum coincide, no descarga nada. Lanza
    ModelDownloadError si el asset no existe, la red falla o el checksum no coincide.

    Args:
        abrir: función que recibe un pedido y devuelve una respuesta; los tests
            inyectan una que no usa la red.
    """
    destino = Path(os.path.expanduser(str(destino)))
    if destino.is_file() and sha256_de(destino) == release.sha256:
        informar('el modelo ya está en %s y su checksum es correcto' % destino)
        return destino

    try:
        with _abrir(API % (release.repository, release.tag), abrir,
                    'application/vnd.github+json') as respuesta:
            datos = json.loads(respuesta.read().decode('utf-8'))
    except OSError as error:
        raise ModelDownloadError('no se pudo consultar el release %s: %s'
                                 % (release.tag, error)) from error

    url_del_asset = next(
        (a['url'] for a in datos.get('assets', []) if a.get('name') == release.asset), None)
    if url_del_asset is None:
        raise ModelDownloadError('el release %s no tiene un asset llamado "%s"'
                                 % (release.tag, release.asset))

    destino.parent.mkdir(parents=True, exist_ok=True)
    temporal = destino.with_name(destino.name + '.parcial')
    resumen = hashlib.sha256()
    try:
        informar('descargando %s ...' % release.asset)
        with _abrir(url_del_asset, abrir, 'application/octet-stream') as respuesta, \
                open(temporal, 'wb') as archivo:
            for bloque in iter(lambda: respuesta.read(TAMANO_DE_BLOQUE), b''):
                archivo.write(bloque)
                resumen.update(bloque)
    except OSError as error:
        temporal.unlink(missing_ok=True)
        raise ModelDownloadError('falló la descarga del modelo: %s' % error) from error

    if resumen.hexdigest() != release.sha256:
        temporal.unlink(missing_ok=True)
        raise ModelDownloadError(
            'el checksum no coincide: se esperaba %s y se obtuvo %s. No se guardó el archivo.'
            % (release.sha256, resumen.hexdigest()))

    os.replace(temporal, destino)
    informar('modelo guardado en %s' % destino)
    return destino


def main(argv=None):
    """Comando `descargar_modelo`: baja el modelo a la ruta que usa el nodo."""
    from ament_index_python.packages import get_package_share_directory

    compartida = Path(get_package_share_directory('bfmc_sign_detection')) / 'config'
    parser = argparse.ArgumentParser(description='Descarga el modelo de señales.')
    parser.add_argument(
        '--destino', help='ruta del archivo; por defecto, el model_path de params.yaml')
    argumentos = parser.parse_args(argv)

    destino = argumentos.destino
    if destino is None:
        with open(compartida / 'params.yaml', encoding='utf-8') as archivo:
            destino = yaml.safe_load(archivo)['sign_detection']['ros__parameters']['model_path']

    try:
        descargar_modelo(cargar_release(compartida / 'model.yaml'), destino)
    except ModelDownloadError as error:
        print('error: %s' % error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
