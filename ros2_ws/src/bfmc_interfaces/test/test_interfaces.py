"""Tests de contrato de bfmc_interfaces.

Si alguien cambia un campo, un tipo o una constante, estos tests fallan y obligan
a avisar a los paquetes que usan el mensaje.
"""
import pytest

from bfmc_interfaces.action import ManeuverExecute
from bfmc_interfaces.msg import (
    HardwareStatus, LaneState, SignDetection, SignDetectionArray, VehicleState,
)


def campos(mensaje):
    return mensaje.get_fields_and_field_types()


def test_lane_state_expone_desvio_rumbo_confianza_e_interseccion():
    assert campos(LaneState) == {
        'header': 'std_msgs/Header',
        'detected': 'boolean',
        'lateral_error_m': 'float',
        'heading_error_rad': 'float',
        'confidence': 'float',
        'intersection_detected': 'boolean',
    }


def test_lane_state_por_defecto_no_detecta_el_carril():
    assert LaneState().detected is False


def test_sign_detection_expone_etiqueta_confianza_distancia_y_caja():
    assert campos(SignDetection) == {
        'label': 'string',
        'confidence': 'float',
        'distance_m': 'float',
        'x_min': 'uint16',
        'y_min': 'uint16',
        'x_max': 'uint16',
        'y_max': 'uint16',
    }


def test_sign_detection_array_contiene_una_lista_de_detecciones():
    assert campos(SignDetectionArray) == {
        'header': 'std_msgs/Header',
        'detections': 'sequence<bfmc_interfaces/SignDetection>',
    }


def test_sign_detection_array_por_defecto_esta_vacio():
    assert list(SignDetectionArray().detections) == []


def test_hardware_status_expone_enlace_watchdog_firmware_secuencia_y_obstaculo():
    assert campos(HardwareStatus) == {
        'header': 'std_msgs/Header',
        'link_up': 'boolean',
        'watchdog_tripped': 'boolean',
        'firmware_version': 'string',
        'last_seq': 'uint32',
        'obstacle_stop': 'boolean',
        'obstacle_distance_m': 'float',
    }


def test_hardware_status_por_defecto_no_informa_ningun_obstaculo():
    assert HardwareStatus().obstacle_stop is False


def test_vehicle_state_define_los_estados_de_la_maquina():
    assert VehicleState.STATE_STARTING == 0
    assert VehicleState.STATE_LANE_FOLLOW == 1
    assert VehicleState.STATE_MANEUVER == 2
    assert VehicleState.STATE_EMERGENCY == 3


def test_vehicle_state_expone_estado_maniobra_y_motivo():
    assert campos(VehicleState) == {
        'header': 'std_msgs/Header',
        'state': 'uint8',
        'active_maneuver': 'string',
        'reason': 'string',
    }


def test_la_accion_pide_maniobra_y_direccion():
    assert campos(ManeuverExecute.Goal) == {
        'maneuver': 'string',
        'direction': 'string',
    }


def test_la_accion_informa_exito_y_mensaje():
    assert campos(ManeuverExecute.Result) == {
        'success': 'boolean',
        'message': 'string',
    }


def test_la_accion_informa_el_tiempo_transcurrido():
    assert campos(ManeuverExecute.Feedback) == {'elapsed_s': 'float'}


@pytest.mark.parametrize('mensaje', [
    LaneState, SignDetection, SignDetectionArray, HardwareStatus, VehicleState,
])
def test_los_mensajes_se_pueden_crear_sin_argumentos(mensaje):
    assert mensaje() is not None
