"""Tests de la lógica del puente: qué trama se envía y qué reporta el ESP32.

Sin ROS, sin puerto serial y con el tiempo simulado.
"""
import json
import math

import pytest

from bfmc_hardware_bridge.hardware_bridge_logic import BridgeConfig, BridgeLogic
from bfmc_hardware_bridge.protocol import SEQ_MAX


def logica(**cambios):
    base = dict(link_timeout_s=0.5, cmd_timeout_s=0.25, max_speed_mps=0.5,
                max_steer_rad=math.radians(25.0), ultrasonic_min_range_m=0.02,
                ultrasonic_max_range_m=4.0)
    return BridgeLogic(BridgeConfig(**{**base, **cambios}))


def trama(logic, now):
    return json.loads(logic.next_frame(now).decode('ascii'))


ULTRA_87 = '{"t":"ultra","id":"front","dist_cm":87,"valid":true}'
ULTRA_SIN_ECO = '{"t":"ultra","id":"front","dist_cm":0,"valid":false}'
STATUS_OK = '{"t":"status","fw":"0.1.0","wd":false,"last_seq":12}'
STATUS_WD = '{"t":"status","fw":"0.1.0","wd":true,"last_seq":13}'


# --- qué se envía ----------------------------------------------------------------------------

def test_un_comando_vigente_se_envia_convertido_al_formato_del_esp32():
    logic = logica()
    logic.on_drive_command(0.25, math.radians(10.0), now=10.0)
    assert trama(logic, now=10.0) == {'t': 'drive', 'seq': 0, 'v': 250, 's': 100}


def test_sin_ningun_comando_se_envia_freno():
    assert trama(logica(), now=0.0)['t'] == 'brake'


def test_un_comando_viejo_se_descarta_y_se_envia_freno():
    logic = logica(cmd_timeout_s=0.25)
    logic.on_drive_command(0.25, 0.0, now=10.0)
    assert trama(logic, now=10.25)['t'] == 'drive'
    assert trama(logic, now=10.5)['t'] == 'brake'


def test_una_emergencia_envia_freno_aunque_haya_un_comando_vigente():
    logic = logica()
    logic.on_drive_command(0.25, 0.0, now=10.0)
    logic.on_safety(True)
    assert trama(logic, now=10.0)['t'] == 'brake'


def test_al_terminar_la_emergencia_se_reanuda_el_comando_vigente():
    logic = logica()
    logic.on_drive_command(0.25, 0.0, now=10.0)
    logic.on_safety(True)
    assert trama(logic, now=10.0)['t'] == 'brake'
    logic.on_safety(False)
    assert trama(logic, now=10.0)['t'] == 'drive'


def test_el_comando_se_recorta_a_los_limites_configurados():
    logic = logica()
    logic.on_drive_command(9.0, math.radians(90.0), now=10.0)
    trama_enviada = trama(logic, now=10.0)
    assert trama_enviada['v'] == 500
    assert trama_enviada['s'] == 250


def test_una_velocidad_nan_se_envia_como_cero():
    logic = logica()
    logic.on_drive_command(math.nan, 0.0, now=10.0)
    assert trama(logic, now=10.0)['v'] == 0


def test_la_secuencia_crece_en_cada_trama():
    logic = logica()
    assert [trama(logic, now=0.0)['seq'] for _ in range(4)] == [0, 1, 2, 3]


def test_la_secuencia_da_la_vuelta_al_llegar_al_maximo_que_lee_el_firmware():
    logic = logica()
    logic._seq = SEQ_MAX
    assert trama(logic, now=0.0)['seq'] == SEQ_MAX
    assert trama(logic, now=0.0)['seq'] == 0


# --- qué reporta el ESP32 --------------------------------------------------------------------

def test_una_lectura_valida_se_convierte_a_metros():
    assert logica().on_serial_line(ULTRA_87, now=0.0) == pytest.approx(0.87)


def test_sin_eco_se_publica_como_camino_despejado():
    assert logica().on_serial_line(ULTRA_SIN_ECO, now=0.0) == math.inf


def test_una_lectura_fuera_del_alcance_se_recorta_al_alcance_del_sensor():
    logic = logica(ultrasonic_max_range_m=0.5)
    assert logic.on_serial_line(ULTRA_87, now=0.0) == 0.5


def test_una_linea_de_estado_no_produce_distancia():
    assert logica().on_serial_line(STATUS_OK, now=0.0) is None


def test_el_estado_del_esp32_se_refleja_en_el_estado_del_hardware():
    logic = logica()
    logic.on_serial_line(STATUS_WD, now=1.0)
    estado = logic.state(now=1.0)
    assert estado.firmware_version == '0.1.0'
    assert estado.watchdog_tripped is True
    assert estado.last_seq == 13


def test_antes_de_recibir_algo_del_esp32_el_enlace_esta_caido():
    estado = logica().state(now=0.0)
    assert estado.link_up is False
    assert estado.firmware_version == ''


def test_un_mensaje_valido_levanta_el_enlace():
    logic = logica()
    logic.on_serial_line(STATUS_OK, now=1.0)
    assert logic.state(now=1.25).link_up is True


def test_sin_mensajes_por_mas_del_tiempo_limite_el_enlace_se_da_por_caido():
    logic = logica(link_timeout_s=0.5)
    logic.on_serial_line(STATUS_OK, now=1.0)
    assert logic.state(now=1.5).link_up is True
    assert logic.state(now=1.75).link_up is False


def test_una_linea_invalida_no_cuenta_como_senal_de_vida():
    logic = logica()
    assert logic.on_serial_line('ruido del cable', now=1.0) is None
    assert logic.state(now=1.0).link_up is False


# --- validación de la configuración ------------------------------------------------------------

@pytest.mark.parametrize('cambio', [
    dict(link_timeout_s=0.0),
    dict(cmd_timeout_s=0.0),
    dict(max_speed_mps=0.0),
    dict(max_steer_rad=0.0),
    dict(ultrasonic_min_range_m=-0.1),
    dict(ultrasonic_min_range_m=4.0),
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        logica(**cambio)
