"""Tests de la descarga del modelo: sin red, con un abridor falso que hace de GitHub."""
import hashlib
import io
import json
from pathlib import Path

import pytest

from bfmc_sign_detection import model_download
from bfmc_sign_detection.model_download import (
    cargar_release, descargar_modelo, ModelDownloadError, ModelRelease, sha256_de,
)

CONTENIDO = b'contenido del modelo ' * 1000
SHA = hashlib.sha256(CONTENIDO).hexdigest()
RELEASE = ModelRelease(repository='org/repo', tag='v1', asset='best.pt', sha256=SHA)
API_RELEASE = 'https://api.github.com/repos/org/repo/releases/tags/v1'
API_ASSET = 'https://api.github.com/repos/org/repo/releases/assets/7'


class Respuesta:
    def __init__(self, datos):
        self._flujo = io.BytesIO(datos)

    def read(self, cantidad=-1):
        return self._flujo.read(cantidad)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class GitHubFalso:
    """Un abridor que imita la API de GitHub y registra qué se pidió."""

    def __init__(self, contenido=CONTENIDO, assets=None, falla_api=False, falla_asset=False):
        self.contenido = contenido
        self.assets = assets if assets is not None else [{'name': 'best.pt', 'url': API_ASSET}]
        self.falla_api = falla_api
        self.falla_asset = falla_asset
        self.pedidos = []

    def __call__(self, pedido):
        self.pedidos.append((pedido.full_url, pedido.get_header('Accept')))
        if pedido.full_url == API_RELEASE:
            if self.falla_api:
                raise OSError('sin conexión')
            return Respuesta(json.dumps({'assets': self.assets}).encode())
        if pedido.full_url == API_ASSET:
            if self.falla_asset:
                raise OSError('conexión cortada')
            return Respuesta(self.contenido)
        raise AssertionError('pedido inesperado: %s' % pedido.full_url)


def silencio(*_):
    pass


# --- descarga correcta ------------------------------------------------------

def test_descarga_el_modelo_y_lo_deja_con_el_checksum_verificado(tmp_path):
    destino = tmp_path / 'best.pt'
    resultado = descargar_modelo(RELEASE, destino, GitHubFalso(), silencio)
    assert resultado == destino
    assert destino.read_bytes() == CONTENIDO
    assert sha256_de(destino) == SHA


def test_crea_las_carpetas_que_faltan(tmp_path):
    destino = tmp_path / 'a' / 'b' / 'best.pt'
    descargar_modelo(RELEASE, destino, GitHubFalso(), silencio)
    assert destino.is_file()


def test_resuelve_el_asset_por_la_api_y_pide_el_binario(tmp_path):
    github = GitHubFalso()
    descargar_modelo(RELEASE, tmp_path / 'best.pt', github, silencio)
    assert github.pedidos == [
        (API_RELEASE, 'application/vnd.github+json'),
        (API_ASSET, 'application/octet-stream'),
    ]


def test_no_deja_archivos_parciales(tmp_path):
    descargar_modelo(RELEASE, tmp_path / 'best.pt', GitHubFalso(), silencio)
    assert [p.name for p in tmp_path.iterdir()] == ['best.pt']


# --- no descarga de más -----------------------------------------------------

def test_si_el_modelo_ya_esta_con_el_checksum_correcto_no_toca_la_red(tmp_path):
    destino = tmp_path / 'best.pt'
    destino.write_bytes(CONTENIDO)
    github = GitHubFalso()
    descargar_modelo(RELEASE, destino, github, silencio)
    assert github.pedidos == []


def test_si_el_modelo_existente_esta_corrupto_se_reemplaza(tmp_path):
    destino = tmp_path / 'best.pt'
    destino.write_bytes(b'archivo a medias')
    descargar_modelo(RELEASE, destino, GitHubFalso(), silencio)
    assert destino.read_bytes() == CONTENIDO


# --- fallas -----------------------------------------------------------------

def test_un_checksum_distinto_se_rechaza_y_no_se_guarda_nada(tmp_path):
    destino = tmp_path / 'best.pt'
    with pytest.raises(ModelDownloadError, match='checksum'):
        descargar_modelo(RELEASE, destino, GitHubFalso(contenido=b'otro contenido'), silencio)
    assert list(tmp_path.iterdir()) == []


def test_un_modelo_corrupto_no_pisa_a_uno_existente_valido_por_otra_via(tmp_path):
    destino = tmp_path / 'best.pt'
    destino.write_bytes(b'version anterior')
    with pytest.raises(ModelDownloadError):
        descargar_modelo(RELEASE, destino, GitHubFalso(contenido=b'basura'), silencio)
    assert destino.read_bytes() == b'version anterior'
    assert [p.name for p in tmp_path.iterdir()] == ['best.pt']


def test_un_release_sin_el_asset_se_informa(tmp_path):
    with pytest.raises(ModelDownloadError, match='best.pt'):
        descargar_modelo(RELEASE, tmp_path / 'best.pt', GitHubFalso(assets=[]), silencio)


def test_una_falla_de_red_al_consultar_el_release_se_informa(tmp_path):
    with pytest.raises(ModelDownloadError, match='release'):
        descargar_modelo(RELEASE, tmp_path / 'best.pt', GitHubFalso(falla_api=True), silencio)


def test_una_falla_de_red_a_mitad_de_la_descarga_no_deja_archivos(tmp_path):
    with pytest.raises(ModelDownloadError):
        descargar_modelo(RELEASE, tmp_path / 'best.pt', GitHubFalso(falla_asset=True), silencio)
    assert list(tmp_path.iterdir()) == []


# --- descripción del release ------------------------------------------------

def test_un_release_con_datos_invalidos_se_rechaza():
    for cambio in (dict(repository='sin_barra'), dict(tag=''), dict(asset=''),
                   dict(sha256='abc'), dict(sha256='G' * 64), dict(sha256=SHA.upper())):
        datos = dict(repository='org/repo', tag='v1', asset='best.pt', sha256=SHA)
        with pytest.raises(ValueError):
            ModelRelease(**{**datos, **cambio})


def test_la_descripcion_del_release_se_lee_de_un_yaml(tmp_path):
    ruta = tmp_path / 'model.yaml'
    ruta.write_text('repository: org/repo\ntag: v1\nasset: best.pt\nsha256: %s\n' % SHA)
    assert cargar_release(ruta) == RELEASE


# --- el comando descargar_modelo --------------------------------------------

def test_el_comando_termina_bien_si_la_descarga_funciona(monkeypatch, tmp_path):
    llamadas = []
    monkeypatch.setattr(model_download, 'descargar_modelo',
                        lambda release, destino: llamadas.append(Path(destino)))
    assert model_download.main(['--destino', str(tmp_path / 'm.pt')]) == 0
    assert llamadas == [tmp_path / 'm.pt']


def test_el_comando_informa_el_error_y_termina_con_codigo_uno(monkeypatch, capsys, tmp_path):
    def falla(release, destino):
        raise ModelDownloadError('sin red')

    monkeypatch.setattr(model_download, 'descargar_modelo', falla)
    assert model_download.main(['--destino', str(tmp_path / 'm.pt')]) == 1
    assert 'sin red' in capsys.readouterr().err
