"""Tests de la lógica de decisión: sin ROS y con el tiempo simulado.

Los instantes son múltiplos de 0.25 para que se representen exactos.
"""
import math

from bfmc_decision.decision_logic import (
    DecisionConfig, DecisionLogic, Drive, HardwareObservation, LaneObservation,
    SignObservation, Source, State, STOPPED,
)
import pytest

CARRIL = Drive(0.25, 0.125)
MANIOBRA = Drive(0.5, -0.25)


def logica(**cambios):
    base = {
        'hardware_timeout_s': 1.5, 'lane_cmd_timeout_s': 0.5,
        'maneuver_cmd_timeout_s': 0.5, 'signs_timeout_s': 0.5,
        'maneuver_request_timeout_s': 1.0, 'maneuver_timeout_s': 15.0, 'maneuver_cooldown_s': 5.0,
        'min_sign_confidence': 0.6, 'stop_distance_m': 1.0, 'intersection_distance_m': 1.5,
        'stop_labels': ('stop',), 'intersection_labels': ('priority', 'roundabout'),
        'intersection_direction': 'left', 'intersection_min_frames': 3,
    }
    return DecisionLogic(DecisionConfig(**{**base, **cambios}))


def entradas(logic, now, obstaculo=False, distancia=math.nan, enlace=True, watchdog=False,
             carril_detectado=True, interseccion=False, comando=CARRIL):
    """Alimenta las entradas habituales, todas con el instante `now`."""
    logic.on_hardware(HardwareObservation(enlace, watchdog, obstaculo, distancia), now)
    logic.on_lane_command(comando, now)
    logic.on_lane(LaneObservation(carril_detectado, interseccion), now)


def senal(logic, now, etiqueta='stop', confianza=0.9, distancia=0.5):
    logic.on_signs([SignObservation(etiqueta, confianza, distancia)], now)


def en_maniobra(logic, now=10.0, etiqueta='stop'):
    """Lleva la lógica a una maniobra activa y devuelve el instante siguiente."""
    entradas(logic, now)
    senal(logic, now, etiqueta)
    assert logic.step(now).maneuver_request is not None
    logic.on_maneuver_goal_response(True, now)
    logic.on_maneuver_command(MANIOBRA, now)
    return now


# --- arranque ---------------------------------------------------------------

def test_sin_ninguna_entrada_el_auto_espera_detenido():
    decision = logica().step(0.0)
    assert decision.state == State.STARTING
    assert decision.command == STOPPED
    assert decision.source == Source.NONE


def test_sin_comando_de_carril_todavia_se_espera():
    logic = logica()
    logic.on_hardware(HardwareObservation(True, False), 10.0)
    assert logic.step(10.0).state == State.STARTING


def test_sin_estado_del_hardware_todavia_se_espera():
    logic = logica()
    logic.on_lane_command(CARRIL, 10.0)
    assert logic.step(10.0).state == State.STARTING


def test_con_las_primeras_entradas_se_sigue_el_carril():
    logic = logica()
    entradas(logic, 10.0)
    decision = logic.step(10.0)
    assert decision.state == State.LANE_FOLLOW
    assert decision.source == Source.LANE
    assert decision.command == CARRIL


# --- emergencia -------------------------------------------------------------

def test_un_obstaculo_que_detuvo_al_esp32_detiene_el_auto_y_dice_a_que_distancia():
    logic = logica()
    entradas(logic, 10.0, obstaculo=True, distancia=0.22)
    decision = logic.step(10.0)
    assert decision.state == State.EMERGENCY
    assert decision.command == STOPPED
    assert 'obstáculo' in decision.reason and '0.22' in decision.reason


def test_un_obstaculo_sin_distancia_conocida_tambien_detiene_el_auto():
    logic = logica()
    entradas(logic, 10.0, obstaculo=True)
    decision = logic.step(10.0)
    assert decision.state == State.EMERGENCY
    assert 'obstáculo' in decision.reason


def test_en_emergencia_el_comando_es_cero_aunque_el_control_pida_avanzar():
    logic = logica()
    entradas(logic, 10.0, obstaculo=True, comando=Drive(0.5, 0.25))
    assert logic.step(10.0).command == STOPPED


def test_ya_no_hace_falta_ningun_estado_de_seguridad_para_conducir():
    logic = logica()
    logic.on_hardware(HardwareObservation(True, False), 10.0)
    logic.on_lane_command(CARRIL, 10.0)
    logic.on_lane(LaneObservation(True, False), 10.0)
    assert logic.step(10.0).state == State.LANE_FOLLOW


def test_un_estado_de_hardware_vencido_es_una_emergencia():
    logic = logica(hardware_timeout_s=1.5)
    entradas(logic, 10.0)
    logic.on_lane_command(CARRIL, 12.0)
    decision = logic.step(12.0)
    assert decision.state == State.EMERGENCY
    assert 'hardware' in decision.reason


def test_un_enlace_caido_es_una_emergencia():
    logic = logica()
    entradas(logic, 10.0, enlace=False)
    decision = logic.step(10.0)
    assert decision.state == State.EMERGENCY
    assert 'enlace' in decision.reason


def test_un_watchdog_cortado_es_una_emergencia():
    logic = logica()
    entradas(logic, 10.0, watchdog=True)
    decision = logic.step(10.0)
    assert decision.state == State.EMERGENCY
    assert 'watchdog' in decision.reason


def test_un_comando_de_carril_vencido_es_una_emergencia():
    logic = logica(lane_cmd_timeout_s=0.5)
    entradas(logic, 10.0)
    logic.on_hardware(HardwareObservation(True, False), 10.75)
    decision = logic.step(10.75)
    assert decision.state == State.EMERGENCY
    assert 'carril' in decision.reason


def test_un_carril_sin_detectar_no_frena_el_auto_por_mucho_que_dure():
    logic = logica()
    entradas(logic, 10.0, carril_detectado=False)
    for instante in (12.25, 20.0, 60.0):
        entradas(logic, instante, carril_detectado=False)
        decision = logic.step(instante)
        assert decision.state == State.LANE_FOLLOW
        assert decision.command == CARRIL


def test_la_emergencia_termina_cuando_las_condiciones_se_normalizan():
    logic = logica()
    entradas(logic, 10.0, obstaculo=True)
    assert logic.step(10.0).state == State.EMERGENCY
    entradas(logic, 10.25, obstaculo=False)
    assert logic.step(10.25).state == State.LANE_FOLLOW


# --- eventos de señal -------------------------------------------------------

def test_un_stop_cercano_pide_la_maniobra_de_stop():
    logic = logica()
    entradas(logic, 10.0)
    senal(logic, 10.0)
    decision = logic.step(10.0)
    assert decision.maneuver_request.name == 'stop'
    assert decision.state == State.MANEUVER
    assert decision.command == STOPPED
    assert decision.active_maneuver == 'stop'


@pytest.mark.parametrize('distancia, confianza, etiqueta', [
    (2.0, 0.9, 'stop'),             # demasiado lejos
    (0.5, 0.2, 'stop'),             # confianza baja
    (math.nan, 0.9, 'stop'),        # distancia desconocida
    (0.5, 0.9, 'estacionamiento'),  # etiqueta sin evento asociado
], ids=['lejos', 'poca_confianza', 'sin_distancia', 'sin_evento'])
def test_una_senal_que_no_cumple_las_condiciones_se_ignora(distancia, confianza, etiqueta):
    logic = logica()
    entradas(logic, 10.0)
    senal(logic, 10.0, etiqueta=etiqueta, confianza=confianza, distancia=distancia)
    decision = logic.step(10.0)
    assert decision.maneuver_request is None
    assert decision.state == State.LANE_FOLLOW


@pytest.mark.parametrize('etiqueta', ['priority', 'roundabout'])
def test_una_senal_de_interseccion_pide_la_interseccion_en_la_direccion_configurada(etiqueta):
    logic = logica(intersection_direction='left')
    entradas(logic, 10.0)
    senal(logic, 10.0, etiqueta=etiqueta, distancia=1.25)
    pedido = logic.step(10.0).maneuver_request
    assert (pedido.name, pedido.direction) == ('intersection', 'left')


def test_la_interseccion_se_activa_a_su_propia_distancia_y_no_a_la_del_stop():
    logic = logica(stop_distance_m=1.0, intersection_distance_m=1.5)
    entradas(logic, 10.0)
    senal(logic, 10.0, etiqueta='priority', distancia=1.25)
    assert logic.step(10.0).maneuver_request is not None


def test_una_interseccion_detectada_en_el_carril_varios_cuadros_seguidos_la_activa():
    logic = logica(intersection_min_frames=3)
    for instante in (10.0, 10.25):
        entradas(logic, instante, interseccion=True)
        assert logic.step(instante).maneuver_request is None
    entradas(logic, 10.5, interseccion=True)
    assert logic.step(10.5).maneuver_request.name == 'intersection'


def test_una_deteccion_interrumpida_en_el_carril_no_alcanza():
    logic = logica(intersection_min_frames=3)
    entradas(logic, 10.0, interseccion=True)
    entradas(logic, 10.25, interseccion=True)
    entradas(logic, 10.5, interseccion=False)
    entradas(logic, 10.75, interseccion=True)
    assert logic.step(10.75).maneuver_request is None


def test_el_stop_tiene_prioridad_sobre_la_interseccion():
    logic = logica()
    entradas(logic, 10.0)
    logic.on_signs([SignObservation('priority', 0.9, 0.5), SignObservation('stop', 0.9, 0.5)],
                   10.0)
    assert logic.step(10.0).maneuver_request.name == 'stop'


def test_las_detecciones_viejas_se_ignoran():
    logic = logica(signs_timeout_s=0.5)
    entradas(logic, 10.0)
    senal(logic, 10.0)
    entradas(logic, 10.75)
    assert logic.step(10.75).maneuver_request is None


def test_el_pedido_de_maniobra_se_emite_una_sola_vez():
    logic = logica()
    entradas(logic, 10.0)
    senal(logic, 10.0)
    assert logic.step(10.0).maneuver_request is not None
    entradas(logic, 10.25)
    senal(logic, 10.25)
    segunda = logic.step(10.25)
    assert segunda.maneuver_request is None
    assert segunda.state == State.MANEUVER


# --- ciclo de vida de una maniobra ------------------------------------------

def test_con_la_maniobra_aceptada_se_usa_su_comando_y_se_ignora_el_carril():
    logic = logica()
    ahora = en_maniobra(logic)
    entradas(logic, ahora + 0.25)
    logic.on_maneuver_command(MANIOBRA, ahora + 0.25)
    decision = logic.step(ahora + 0.25)
    assert decision.state == State.MANEUVER
    assert decision.source == Source.MANEUVER
    assert decision.command == MANIOBRA


def test_mientras_el_servidor_no_responde_el_auto_permanece_detenido():
    logic = logica()
    entradas(logic, 10.0)
    senal(logic, 10.0)
    logic.step(10.0)
    entradas(logic, 10.25)
    decision = logic.step(10.25)
    assert decision.state == State.MANEUVER
    assert decision.command == STOPPED


def test_una_maniobra_sin_comando_vigente_deja_el_auto_detenido():
    logic = logica(maneuver_cmd_timeout_s=0.5)
    ahora = en_maniobra(logic)
    entradas(logic, ahora + 0.75)
    decision = logic.step(ahora + 0.75)
    assert decision.state == State.MANEUVER
    assert decision.command == STOPPED


def test_al_terminar_la_maniobra_se_vuelve_a_seguir_el_carril():
    logic = logica()
    ahora = en_maniobra(logic)
    logic.on_maneuver_result(ahora + 1.0)
    entradas(logic, ahora + 1.0)
    decision = logic.step(ahora + 1.0)
    assert decision.state == State.LANE_FOLLOW
    assert decision.command == CARRIL


def test_durante_una_maniobra_no_rige_la_emergencia_por_comando_de_carril_vencido():
    logic = logica(lane_cmd_timeout_s=0.5)
    ahora = en_maniobra(logic)
    logic.on_hardware(HardwareObservation(True, False), ahora + 2.0)
    logic.on_maneuver_command(MANIOBRA, ahora + 2.0)
    assert logic.step(ahora + 2.0).state == State.MANEUVER


def test_la_espera_entre_maniobras_evita_repetir_la_misma_senal():
    logic = logica(maneuver_cooldown_s=5.0)
    ahora = en_maniobra(logic)
    logic.on_maneuver_result(ahora + 1.0)
    entradas(logic, ahora + 2.0)
    senal(logic, ahora + 2.0)
    assert logic.step(ahora + 2.0).maneuver_request is None
    entradas(logic, ahora + 6.25)
    senal(logic, ahora + 6.25)
    assert logic.step(ahora + 6.25).maneuver_request.name == 'stop'


def test_una_maniobra_rechazada_vuelve_al_carril_y_respeta_la_espera():
    logic = logica(maneuver_cooldown_s=5.0)
    entradas(logic, 10.0)
    senal(logic, 10.0)
    logic.step(10.0)
    logic.on_maneuver_goal_response(False, 10.25)
    entradas(logic, 10.5)
    senal(logic, 10.5)
    decision = logic.step(10.5)
    assert decision.state == State.LANE_FOLLOW
    assert decision.maneuver_request is None


def test_si_el_servidor_no_responde_a_tiempo_se_cancela_y_hay_emergencia():
    logic = logica(maneuver_request_timeout_s=1.0)
    entradas(logic, 10.0)
    senal(logic, 10.0)
    logic.step(10.0)
    entradas(logic, 11.25)
    decision = logic.step(11.25)
    assert decision.state == State.EMERGENCY
    assert decision.cancel_maneuver is True
    entradas(logic, 11.5)
    assert logic.step(11.5).state == State.LANE_FOLLOW


def test_una_maniobra_que_supera_su_tiempo_maximo_se_cancela():
    logic = logica(maneuver_timeout_s=3.0)
    ahora = en_maniobra(logic)
    entradas(logic, ahora + 3.25)
    logic.on_maneuver_command(MANIOBRA, ahora + 3.25)
    decision = logic.step(ahora + 3.25)
    assert decision.state == State.EMERGENCY
    assert decision.cancel_maneuver is True


def test_un_obstaculo_durante_la_maniobra_la_cancela_una_sola_vez():
    logic = logica()
    ahora = en_maniobra(logic)
    entradas(logic, ahora + 0.25, obstaculo=True, distancia=0.22)
    primera = logic.step(ahora + 0.25)
    assert primera.state == State.EMERGENCY
    assert primera.cancel_maneuver is True
    entradas(logic, ahora + 0.5, obstaculo=True, distancia=0.22)
    assert logic.step(ahora + 0.5).cancel_maneuver is False


def test_el_resultado_tardio_de_una_maniobra_cancelada_se_ignora():
    logic = logica()
    ahora = en_maniobra(logic)
    entradas(logic, ahora + 0.25, obstaculo=True)
    logic.step(ahora + 0.25)
    logic.on_maneuver_result(ahora + 0.5)
    entradas(logic, ahora + 0.75)
    assert logic.step(ahora + 0.75).state == State.LANE_FOLLOW


def test_una_respuesta_del_servidor_sin_pedido_pendiente_se_ignora():
    logic = logica()
    entradas(logic, 10.0)
    logic.on_maneuver_goal_response(True, 10.0)
    assert logic.step(10.0).state == State.LANE_FOLLOW


# --- una sola fuente por ciclo ----------------------------------------------

def test_el_comando_siempre_sale_de_una_sola_fuente_coherente_con_el_estado():
    logic = logica()
    decisiones = []

    decisiones.append(logic.step(9.0))                    # arranque
    entradas(logic, 10.0)
    decisiones.append(logic.step(10.0))                   # carril
    senal(logic, 10.0)
    decisiones.append(logic.step(10.0))                   # pide la maniobra
    logic.on_maneuver_goal_response(True, 10.25)
    entradas(logic, 10.25)
    logic.on_maneuver_command(MANIOBRA, 10.25)
    decisiones.append(logic.step(10.25))                  # maniobra
    entradas(logic, 10.5, obstaculo=True)
    decisiones.append(logic.step(10.5))                   # emergencia

    for decision in decisiones:
        if decision.source == Source.LANE:
            assert decision.state == State.LANE_FOLLOW and decision.command == CARRIL
        elif decision.source == Source.MANEUVER:
            assert decision.state == State.MANEUVER and decision.command == MANIOBRA
        else:
            assert decision.command == STOPPED
    assert [d.source for d in decisiones] == [
        Source.NONE, Source.LANE, Source.NONE, Source.MANEUVER, Source.NONE]


# --- validación de la configuración -----------------------------------------

@pytest.mark.parametrize('cambio', [
    {'hardware_timeout_s': -1.0},
    {'lane_cmd_timeout_s': 0.0},
    {'signs_timeout_s': 0.0},
    {'maneuver_timeout_s': 0.0},
    {'maneuver_cooldown_s': -1.0},
    {'min_sign_confidence': 1.5},
    {'stop_distance_m': 0.0},
    {'intersection_distance_m': -1.0},
    {'intersection_direction': 'atras'},
    {'intersection_min_frames': 0},
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        logica(**cambio)
