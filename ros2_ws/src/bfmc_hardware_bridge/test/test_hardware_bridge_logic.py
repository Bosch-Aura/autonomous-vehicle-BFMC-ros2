"""Tests de la lógica del puente: qué se envía al ESP32 y qué estado reporta.

Sin ROS, sin puerto serial y con el tiempo simulado.
"""
import math

from bfmc_hardware_bridge.hardware_bridge_logic import BridgeConfig, BridgeLogic
from bfmc_hardware_bridge.units import Esp32Limits
import pytest

LIMITES = Esp32Limits(max_speed_mps=0.5, max_steer_rad=math.radians(25.0), duty_start=140,
                      duty_max=255, servo_left=50, servo_center=105, servo_right=160)

ARRANQUE_DEL_ESP32 = [
    'EVENT:STATE_CHANGED:DISARMED',
    'EVENT:SYSTEM_READY',
]


def logica(**cambios):
    base = {'link_timeout_s': 0.5, 'cmd_timeout_s': 0.25, 'arm_retry_s': 0.5,
            'limits': LIMITES}
    return BridgeLogic(BridgeConfig(**{**base, **cambios}))


def lineas(logic, now):
    return logic.next_frames(now).decode('ascii').splitlines()


def recibir(logic, now, *eventos):
    for evento in eventos:
        logic.on_serial_line(evento, now)


def lista_para_andar(**cambios):
    """Un puente con el ESP32 ya armado y en RUNNING."""
    logic = logica(**cambios)
    logic.on_port_opened()
    recibir(logic, 0.0, 'EVENT:STATE_CHANGED:RUNNING')
    return logic


# --- velocidad y dirección -----------------------------------------------------------------

def test_un_comando_vigente_se_envia_en_las_unidades_del_esp32():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, math.radians(25.0), now=10.0)
    assert lineas(logic, now=10.0) == ['C:SET_DIR:1', 'C:SET_STEER:50', 'C:SET_SPEED:255']


def test_velocidad_y_direccion_se_reenvian_en_cada_ciclo_como_heartbeat():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, 0.0, now=10.0)
    lineas(logic, now=10.0)
    assert lineas(logic, now=10.05) == ['C:SET_STEER:105', 'C:SET_SPEED:255']


def test_el_sentido_solo_se_envia_cuando_cambia_y_antes_que_la_velocidad():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, 0.0, now=10.0)
    assert lineas(logic, now=10.0)[0] == 'C:SET_DIR:1'
    logic.on_drive_command(-0.5, 0.0, now=10.1)
    assert lineas(logic, now=10.1) == ['C:SET_DIR:0', 'C:SET_STEER:105', 'C:SET_SPEED:255']
    logic.on_drive_command(-0.5, 0.0, now=10.2)
    assert 'C:SET_DIR:0' not in lineas(logic, now=10.2)


def test_parar_no_cambia_el_sentido():
    logic = lista_para_andar()
    logic.on_drive_command(-0.5, 0.0, now=10.0)
    lineas(logic, now=10.0)
    logic.on_drive_command(0.0, 0.0, now=10.1)
    assert lineas(logic, now=10.1) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


def test_sin_ningun_comando_se_envia_velocidad_cero_y_direccion_al_centro():
    assert lineas(lista_para_andar(), now=0.0) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


def test_un_comando_viejo_se_descarta_y_se_frena():
    logic = lista_para_andar(cmd_timeout_s=0.25)
    logic.on_drive_command(0.5, math.radians(25.0), now=10.0)
    assert 'C:SET_SPEED:255' in lineas(logic, now=10.25)
    assert lineas(logic, now=10.5) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


# --- obstáculo del ESP32 ------------------------------------------------------------------

def test_un_obstaculo_del_esp32_se_informa_con_su_distancia():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.obstacle_cm, estado.esp32_state) == (True, 22, 'STOPPED')


def test_al_despejarse_el_obstaculo_ya_no_se_informa():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE')
    recibir(logic, 2.0, 'EVENT:OBSTACLE_CLEARED', 'EVENT:STATE_AUTO_TRANSITION:STOPPED->RUNNING')
    estado = logic.state(now=2.0)
    assert (estado.obstacle, estado.obstacle_cm, estado.esp32_state) == (False, None, 'RUNNING')


def test_la_transicion_a_stopped_por_obstaculo_lo_informa_aunque_no_llegue_el_aviso():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.obstacle_cm) == (True, None)


def test_un_stopped_pedido_por_servicio_no_es_un_obstaculo():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:SYS_STOP')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.esp32_state) == (False, 'STOPPED')


def test_una_distancia_ilegible_se_informa_como_desconocida():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:abc')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.obstacle_cm) == (True, None)


@pytest.mark.parametrize('evento', [
    'EVENT:WATCHDOG_TIMEOUT', 'EVENT:ESTOP_TRIGGERED:GPIO', 'EVENT:STATE_CHANGED:RUNNING',
    'EVENT:STATE_CHANGED:DISARMED', 'EVENT:SYSTEM_READY',
])
def test_un_cambio_de_estado_borra_el_obstaculo(evento):
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE')
    recibir(logic, 2.0, evento)
    assert logic.state(now=2.0).obstacle is False


def test_al_reabrir_el_puerto_se_olvida_el_obstaculo():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22')
    logic.on_port_opened()
    assert logic.state(now=1.0).obstacle is False


def test_con_un_obstaculo_el_puente_no_manda_ningun_freno_y_sigue_con_el_comando():
    # Frena el firmware. El puente sigue enviando el comando vigente: apenas el ESP32 se
    # despeje, el auto vuelve a andar con el próximo ciclo (el firmware lo pide así).
    logic = lista_para_andar()
    logic.on_drive_command(0.5, 0.0, now=10.0)
    recibir(logic, 10.0, 'EVENT:OBSTACLE_DETECTED:22',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE')
    enviadas = lineas(logic, now=10.0) + lineas(logic, now=10.05)
    assert 'E:BRAKE_NOW:0' not in enviadas
    assert 'C:SET_SPEED:255' in enviadas


# --- armado ---------------------------------------------------------------------------------

def test_al_abrir_el_puerto_se_arma_sin_configurar_ningun_modo():
    # El firmware no tiene modos: M:SYS_MODE no existe y lo descartaría sin responder.
    logic = logica()
    logic.on_port_opened()
    assert lineas(logic, now=0.0)[0] == 'M:SYS_ARM:0'
    assert not any('SYS_MODE' in linea for linea in lineas(logic, now=1.0))


def test_se_arma_y_se_reintenta_hasta_la_confirmacion():
    logic = logica(arm_retry_s=0.5)
    logic.on_port_opened()
    assert lineas(logic, now=0.0)[0] == 'M:SYS_ARM:0'
    assert 'M:SYS_ARM:0' not in lineas(logic, now=0.25)
    assert lineas(logic, now=0.5)[0] == 'M:SYS_ARM:0'
    recibir(logic, 0.55, 'EVENT:CMD_EXECUTED:SYS_ARM')
    assert lineas(logic, now=1.0) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


def test_nunca_se_envian_dos_comandos_de_gestion_en_el_mismo_ciclo():
    # El firmware guarda un solo comando de gestión: el segundo pisaría al primero.
    logic = logica()
    logic.on_port_opened()
    for paso in range(20):
        now = paso * 0.05
        gestion = [linea for linea in lineas(logic, now) if linea.startswith('M:')]
        assert len(gestion) <= 1
        if paso == 3:
            recibir(logic, now, 'EVENT:CMD_REJECTED:SYS_ARM:ESTOP_ACTIVE')


def test_si_el_esp32_ya_estaba_andando_no_se_lo_arma_de_nuevo():
    logic = logica()
    logic.on_port_opened()
    recibir(logic, 0.0, 'EVENT:STATE_CHANGED:RUNNING')
    assert not any(linea.startswith('M:') for linea in lineas(logic, now=0.0))


def test_un_fault_posterior_no_se_rearma_solo():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:WATCHDOG_TIMEOUT', 'EVENT:STATE_CHANGED:FAULT')
    for paso in range(40):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=1.0 + paso * 0.05)
    assert logic.state(now=1.0).fault is True


@pytest.mark.parametrize('evento', [
    'EVENT:WATCHDOG_TIMEOUT',
    'EVENT:FAULT_TRIGGERED:ULTRASONIC_OBSTACLE',
    'EVENT:ESTOP_TRIGGERED:GPIO',
    'EVENT:STATE_CHANGED:FAULT',
])
def test_cada_motivo_de_fault_se_refleja_en_el_estado(evento):
    logic = lista_para_andar()
    recibir(logic, 1.0, evento)
    assert logic.state(now=1.0).fault is True


def test_pedir_armado_saca_al_esp32_del_fault():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_CHANGED:FAULT')
    logic.request_arm()
    assert lineas(logic, now=1.1)[0] == 'M:SYS_ARM:0'
    recibir(logic, 1.2, 'EVENT:CMD_EXECUTED:SYS_ARM', 'EVENT:STATE_CHANGED:ARMED')
    assert logic.state(now=1.2).fault is False
    assert not any(linea.startswith('M:') for linea in lineas(logic, now=1.25))


def test_un_armado_rechazado_por_el_e_stop_se_sigue_reintentando():
    logic = logica(arm_retry_s=0.5)
    logic.on_port_opened()
    lineas(logic, now=0.0)
    recibir(logic, 0.1, 'EVENT:CMD_REJECTED:SYS_ARM:ESTOP_ACTIVE')
    assert lineas(logic, now=0.5)[0] == 'M:SYS_ARM:0'


def test_si_el_esp32_se_reinicia_se_vuelve_a_armar():
    logic = lista_para_andar()
    logic.on_drive_command(-0.5, 0.0, now=1.0)
    lineas(logic, now=1.0)
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    assert lineas(logic, now=2.0)[0] == 'M:SYS_ARM:0'


def test_si_el_esp32_arranca_armado_no_se_lo_vuelve_a_armar():
    # Con -DBOOT_ARMED el firmware arranca ARMED y lo informa antes de SYSTEM_READY.
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_CHANGED:ARMED',
            'EVENT:STATE_AUTO_TRANSITION:ARMED->RUNNING', 'EVENT:SYSTEM_READY')
    assert logic.state(now=1.0).esp32_state == 'RUNNING'
    assert not any(linea.startswith('M:') for linea in lineas(logic, now=1.0))


def test_despues_de_un_reinicio_se_reenvia_el_sentido():
    # El firmware arranca hacia adelante: el sentido recordado ya no vale.
    logic = lista_para_andar()
    logic.on_drive_command(-0.5, 0.0, now=1.0)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, *ARRANQUE_DEL_ESP32)
    recibir(logic, 1.05, 'EVENT:STATE_CHANGED:RUNNING')
    logic.on_drive_command(-0.5, 0.0, now=1.1)
    assert 'C:SET_DIR:0' in lineas(logic, now=1.1)


def test_stopped_cuenta_como_armado_y_no_se_lo_rearma():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE',
            'EVENT:STATE_CHANGED:STOPPED')
    assert logic.state(now=1.0).esp32_state == 'STOPPED'
    assert logic.state(now=1.0).fault is False
    assert not any(linea.startswith('M:') for linea in lineas(logic, now=1.0))


def test_la_transicion_automatica_toma_el_estado_de_destino():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:SYS_STOP')
    assert logic.state(now=1.0).esp32_state == 'STOPPED'
    recibir(logic, 2.0, 'EVENT:STATE_AUTO_TRANSITION:STOPPED->RUNNING')
    assert logic.state(now=2.0).esp32_state == 'RUNNING'


def test_si_el_esp32_arranca_con_un_obstaculo_queda_armado_en_stopped():
    # Con un obstáculo delante, al armar el ESP32 pasa por ARMED y RUNNING a STOPPED.
    logic = logica()
    logic.on_port_opened()
    recibir(logic, 0.0, *ARRANQUE_DEL_ESP32)
    assert lineas(logic, now=0.0)[0] == 'M:SYS_ARM:0'
    recibir(logic, 0.05, 'EVENT:CMD_EXECUTED:SYS_ARM', 'EVENT:STATE_CHANGED:ARMED',
            'EVENT:STATE_AUTO_TRANSITION:ARMED->RUNNING',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE',
            'EVENT:STATE_CHANGED:STOPPED', 'EVENT:CMD_EXECUTED:SYS_ARM')
    assert logic.state(now=0.05).esp32_state == 'STOPPED'
    assert not any(linea.startswith('M:') for linea in lineas(logic, now=1.0))


# --- detener y arrancar (STOPPED) -------------------------------------------------------------

def gestion(logic, now):
    return [linea for linea in lineas(logic, now) if linea.startswith('M:')]


def test_sin_pedidos_no_se_envia_ni_stop_ni_resume():
    logic = lista_para_andar()
    assert gestion(logic, now=1.0) == []
    assert logic.stop_requested() is None


def test_detener_envia_sys_stop_y_lo_reintenta_hasta_la_confirmacion():
    logic = lista_para_andar(arm_retry_s=0.5)
    logic.request_stop(True)
    assert gestion(logic, now=1.0) == ['M:SYS_STOP:0']
    assert gestion(logic, now=1.25) == []
    assert gestion(logic, now=1.5) == ['M:SYS_STOP:0']
    recibir(logic, 1.55, 'EVENT:CMD_RECEIVED:SYS_STOP', 'EVENT:CMD_EXECUTED:SYS_STOP',
            'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:SYS_STOP',
            'EVENT:STATE_CHANGED:STOPPED')
    assert gestion(logic, now=2.5) == []
    assert logic.state(now=2.5).stopped is True


def test_arrancar_envia_sys_resume_hasta_la_confirmacion():
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_RECEIVED:SYS_STOP')
    logic.request_stop(False)
    assert gestion(logic, now=1.1) == ['M:SYS_RESUME:0']
    recibir(logic, 1.15, 'EVENT:CMD_RECEIVED:SYS_RESUME',
            'EVENT:STATE_AUTO_TRANSITION:STOPPED->RUNNING')
    assert gestion(logic, now=2.0) == []
    assert logic.state(now=2.0).esp32_state == 'RUNNING'


def test_una_confirmacion_del_pedido_anterior_no_cuenta():
    # Se pidió detener y enseguida arrancar: el CMD_RECEIVED:SYS_STOP que llega
    # tarde no confirma el arranque.
    logic = lista_para_andar(arm_retry_s=0.5)
    logic.request_stop(True)
    lineas(logic, now=1.0)
    logic.request_stop(False)
    lineas(logic, now=1.05)
    recibir(logic, 1.1, 'EVENT:CMD_RECEIVED:SYS_STOP')
    assert gestion(logic, now=1.6) == ['M:SYS_RESUME:0']


def test_repetir_el_mismo_pedido_no_reenvia_nada():
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_RECEIVED:SYS_STOP')
    logic.request_stop(True)
    assert gestion(logic, now=1.1) == []


def test_si_el_esp32_se_reinicia_se_le_vuelve_a_enviar_el_stop():
    # El ESP32 olvida el SYS_STOP al reiniciarse.
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_RECEIVED:SYS_STOP')
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    enviados = gestion(logic, now=2.0) + gestion(logic, now=2.05)
    assert enviados == ['M:SYS_STOP:0', 'M:SYS_ARM:0']


def test_tras_un_reinicio_un_pedido_de_arrancar_no_se_reenvia():
    logic = lista_para_andar()
    logic.request_stop(False)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_RECEIVED:SYS_RESUME')
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    assert gestion(logic, now=2.0) == ['M:SYS_ARM:0']


def test_stop_y_armado_nunca_salen_en_el_mismo_ciclo():
    logic = logica()
    logic.on_port_opened()
    logic.request_stop(True)
    for paso in range(20):
        assert len(gestion(logic, now=paso * 0.05)) <= 1


def test_al_cerrar_se_frena_y_se_desarma():
    assert logica().shutdown_frames() == b'C:SET_SPEED:0\nM:SYS_DISARM:0\n'


# --- enlace ------------------------------------------------------------------------------------

def test_antes_de_recibir_algo_del_esp32_el_enlace_esta_caido():
    estado = logica().state(now=0.0)
    assert estado.link_up is False
    assert estado.esp32_state == ''


def test_un_evento_levanta_el_enlace():
    logic = logica()
    recibir(logic, 1.0, 'EVENT:CMD_RECEIVED:SET_SPEED:0')
    assert logic.state(now=1.25).link_up is True


def test_sin_eventos_por_mas_del_tiempo_limite_el_enlace_se_da_por_caido():
    logic = logica(link_timeout_s=0.5)
    recibir(logic, 1.0, 'EVENT:CMD_RECEIVED:SET_SPEED:0')
    assert logic.state(now=1.5).link_up is True
    assert logic.state(now=1.75).link_up is False


@pytest.mark.parametrize('linea', ['[MotorTask] Motor task started', 'ruido', ''])
def test_la_depuracion_y_el_ruido_no_cuentan_como_senal_de_vida(linea):
    logic = logica()
    recibir(logic, 1.0, linea)
    assert logic.state(now=1.0).link_up is False


def test_el_estado_del_esp32_se_refleja_en_el_estado():
    logic = logica()
    recibir(logic, 1.0, 'EVENT:STATE_CHANGED:ARMED',
            'EVENT:STATE_AUTO_TRANSITION:ARMED->RUNNING')
    assert logic.state(now=1.0).esp32_state == 'RUNNING'


def test_al_reabrir_el_puerto_se_olvida_lo_que_se_sabia_del_esp32():
    logic = lista_para_andar()
    logic.on_port_opened()
    estado = logic.state(now=0.0)
    assert (estado.link_up, estado.esp32_state) == (False, '')


# --- configuración ---------------------------------------------------------------------------

@pytest.mark.parametrize('cambios', [
    {'link_timeout_s': 0.0},
    {'cmd_timeout_s': -1.0},
    {'arm_retry_s': 0.0},
])
def test_una_configuracion_invalida_se_rechaza(cambios):
    with pytest.raises(ValueError):
        logica(**cambios)
