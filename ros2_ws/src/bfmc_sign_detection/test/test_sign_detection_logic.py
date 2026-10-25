"""Tests de la lógica de señales con un detector falso: no necesitan el modelo ni YOLO."""
import math

from bfmc_sign_detection.sign_detection_logic import (
    parse_class_map, RawDetection, SignConfig, SignProcessor,
)
import numpy as np
import pytest

ANCHO = 200
ALTO = 100
MAPA = {'stop': 'stop', 'onewayroad': 'one_way', 'priority': 'priority'}


class DetectorFalso:

    def __init__(self, detecciones):
        self.detecciones = detecciones

    def detect(self, bgr):
        return self.detecciones


def configuracion(**cambios):
    base = {'class_map': MAPA, 'min_confidence': 0.5, 'depth_window_fraction': 0.5,
            'min_depth_m': 0.2, 'max_depth_m': 6.0, 'min_depth_pixels': 10}
    return SignConfig(**{**base, **cambios})


def procesar(detecciones, profundidad=None, **cambios):
    imagen = np.zeros((ALTO, ANCHO, 3), dtype=np.uint8)
    return SignProcessor(DetectorFalso(detecciones), configuracion(**cambios)).process(
        imagen, profundidad)


def stop(confianza=0.9, caja=(80, 30, 120, 70)):
    return RawDetection('stop', confianza, *caja)


def profundidad(valor, alto=ALTO, ancho=ANCHO):
    return np.full((alto, ancho), valor, dtype=np.float32)


# --- qué se publica ---------------------------------------------------------

def test_una_deteccion_valida_se_publica_con_su_etiqueta_y_su_caja():
    senal, = procesar([stop()], profundidad(2.0))
    assert (senal.label, senal.confidence) == ('stop', pytest.approx(0.9))
    assert (senal.x_min, senal.y_min, senal.x_max, senal.y_max) == (80, 30, 120, 70)


def test_la_clase_del_modelo_se_traduce_a_la_etiqueta_del_sistema():
    senal, = procesar([RawDetection('onewayroad', 0.9, 80, 30, 120, 70)], profundidad(2.0))
    assert senal.label == 'one_way'


def test_una_clase_que_no_esta_en_el_mapa_se_descarta():
    assert procesar([RawDetection('semaforo_raro', 0.99, 80, 30, 120, 70)]) == []


def test_una_deteccion_con_poca_confianza_se_descarta():
    assert procesar([stop(confianza=0.3)]) == []


def test_sin_detecciones_no_se_publica_nada():
    assert procesar([]) == []


def test_las_senales_se_entregan_de_mayor_a_menor_confianza():
    detecciones = [stop(0.6), RawDetection('priority', 0.95, 10, 10, 50, 50), stop(0.8)]
    assert [s.confidence for s in procesar(detecciones)] == pytest.approx([0.95, 0.8, 0.6])


# --- cajas ------------------------------------------------------------------

def test_una_caja_que_se_sale_de_la_imagen_se_recorta():
    senal, = procesar([stop(caja=(-10.5, -5.0, 250.2, 120.0))], profundidad(2.0))
    assert (senal.x_min, senal.y_min, senal.x_max, senal.y_max) == (0, 0, ANCHO, ALTO)


@pytest.mark.parametrize('caja', [
    (120, 30, 80, 70),       # invertida
    (80, 30, 80, 70),        # sin ancho
    (300, 10, 400, 50),      # totalmente afuera
], ids=['invertida', 'sin_ancho', 'afuera'])
def test_una_caja_sin_area_dentro_de_la_imagen_se_descarta(caja):
    assert procesar([stop(caja=caja)]) == []


# --- distancia --------------------------------------------------------------

def test_la_distancia_es_la_profundidad_de_la_ventana_central():
    senal, = procesar([stop()], profundidad(2.0))
    assert senal.distance_m == pytest.approx(2.0)


def test_la_mediana_resiste_a_los_valores_cercanos_erroneos():
    mapa = profundidad(2.0)
    mapa[40:60, 90:96] = 0.5           # 30 % de la ventana con una medición espuria
    senal, = procesar([stop()], mapa)
    assert senal.distance_m == pytest.approx(2.0)


def test_los_valores_invalidos_no_cuentan():
    mapa = profundidad(2.0)
    mapa[40:60, 90:100] = np.nan
    mapa[40:60, 100:104] = 0.0
    senal, = procesar([stop()], mapa)
    assert senal.distance_m == pytest.approx(2.0)


def test_los_valores_fuera_del_rango_valido_no_cuentan():
    mapa = profundidad(9.0)            # más lejos que max_depth_m
    mapa[45:55, 95:105] = 2.5
    senal, = procesar([stop()], mapa, min_depth_pixels=20)
    assert senal.distance_m == pytest.approx(2.5)


def test_con_muy_pocos_valores_validos_la_distancia_es_desconocida():
    mapa = np.full((ALTO, ANCHO), np.nan, dtype=np.float32)
    mapa[48:50, 98:101] = 2.0          # 6 píxeles válidos
    senal, = procesar([stop()], mapa, min_depth_pixels=10)
    assert math.isnan(senal.distance_m)


def test_sin_profundidad_la_distancia_es_desconocida_pero_la_senal_se_publica():
    senal, = procesar([stop()], None)
    assert math.isnan(senal.distance_m)
    assert senal.label == 'stop'


def test_solo_se_usa_la_ventana_central_de_la_caja():
    mapa = profundidad(1.0)            # el borde de la caja está a 1 m
    mapa[40:60, 90:110] = 3.0          # el centro, a 3 m
    senal, = procesar([stop()], mapa, depth_window_fraction=0.4)
    assert senal.distance_m == pytest.approx(3.0)


def test_una_ventana_mas_grande_incluye_el_borde_de_la_caja():
    mapa = profundidad(1.0)
    mapa[40:60, 90:110] = 3.0
    senal, = procesar([stop()], mapa, depth_window_fraction=1.0)
    assert senal.distance_m == pytest.approx(1.0)


def test_una_profundidad_de_otra_resolucion_escala_las_cajas():
    """Las cajas se escalan cuando la profundidad tiene otra resolución.

    La profundidad tiene la mitad de resolución. Solo el lugar donde cae el centro de
    la caja (100, 50) escalado a (50, 25) vale 3 m; el resto vale 1 m.
    """
    mapa = profundidad(1.0, alto=ALTO // 2, ancho=ANCHO // 2)
    mapa[20:30, 45:55] = 3.0
    senal, = procesar([stop()], mapa)
    assert senal.distance_m == pytest.approx(3.0)


def test_cada_senal_tiene_su_propia_distancia():
    mapa = profundidad(5.0)
    mapa[10:50, 10:50] = 1.5
    cerca = RawDetection('priority', 0.9, 10, 10, 50, 50)
    lejos = stop(0.8)
    resultado = {s.label: s.distance_m for s in procesar([cerca, lejos], mapa)}
    assert resultado['priority'] == pytest.approx(1.5)
    assert resultado['stop'] == pytest.approx(5.0)


# --- mapa de clases ---------------------------------------------------------

def test_el_mapa_de_clases_se_lee_de_pares_clase_etiqueta():
    assert parse_class_map(['stop:stop', 'onewayroad: one_way ']) == {
        'stop': 'stop', 'onewayroad': 'one_way'}


@pytest.mark.parametrize('entradas', [
    ['stop'],
    ['stop:'],
    [':stop'],
    ['stop:stop', 'stop:otra'],
])
def test_una_entrada_invalida_del_mapa_se_rechaza(entradas):
    with pytest.raises(ValueError):
        parse_class_map(entradas)


# --- validación de la configuración -----------------------------------------

@pytest.mark.parametrize('cambio', [
    {'class_map': {}},
    {'min_confidence': 1.5},
    {'depth_window_fraction': 0.0},
    {'depth_window_fraction': 1.5},
    {'min_depth_m': 6.0},
    {'min_depth_m': -1.0},
    {'min_depth_pixels': 0},
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        configuracion(**cambio)
