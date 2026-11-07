"""Tests de la lógica del puente: qué se envía al ESP32 y qué estado reporta.

Sin ROS, sin puerto serial y con el tiempo simulado. Los eventos son los del
firmware actual de Bosch-Aura/embedded (docs/BRAIN_TEAM_PROTOCOL.md).
"""
import math

from bfmc_hardware_bridge.hardware_bridge_logic import BridgeConfig, BridgeLogic
from bfmc_hardware_bridge.units import Esp32Limits
import pytest

LIMITES = Esp32Limits(max_speed_mps=0.5, max_steer_rad=math.radians(30.0), duty_start=140,
                      duty_max=255, servo_left=50, servo_center=105, servo_right=160)

# Al arrancar, el firmware informa su estado inicial y después que terminó.
ARRANQUE_DEL_ESP32 = [
    'EVENT:STATE_CHANGED:DISARMED',
    'EVENT:SYSTEM_READY',
]


def logica(**cambios):
    base = {'link_timeout_s': 1.0, 'cmd_timeout_s': 0.25, 'arm_retry_s': 0.5,
            'heartbeat_period_s': 0.3, 'auto_obstacle_reset': True,
            'obstacle_reblock_window_s': 3.0, 'limits': LIMITES}
    return BridgeLogic(BridgeConfig(**{**base, **cambios}))


def lineas(logic, now):
    return logic.next_frames(now).decode('ascii').splitlines()


def gestion(logic, now):
    return [linea for linea in lineas(logic, now) if linea.startswith('M:')]


def recibir(logic, now, *eventos):
    for evento in eventos:
        logic.on_serial_line(evento, now)


def lista_para_andar(**cambios):
    """Un puente con el ESP32 ya armado y en RUNNING, sin ningún stop pedido."""
    logic = logica(**cambios)
    logic.on_port_opened()
    recibir(logic, 0.0, 'EVENT:STATE_CHANGED:RUNNING')
    return logic


def control(logic, now):
    """Devuelve las líneas C: del ciclo, sin el latido."""
    return [linea for linea in lineas(logic, now) if not linea.startswith('M:')]


# --- velocidad y dirección -----------------------------------------------------------------

def test_un_comando_vigente_se_envia_en_las_unidades_del_esp32():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, math.radians(30.0), now=10.0)
    assert control(logic, now=10.0) == ['C:SET_DIR:1', 'C:SET_STEER:50', 'C:SET_SPEED:255']


def test_velocidad_y_direccion_se_reenvian_en_cada_ciclo_para_el_watchdog():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, 0.0, now=10.0)
    lineas(logic, now=10.0)
    assert control(logic, now=10.05) == ['C:SET_STEER:105', 'C:SET_SPEED:255']


def test_el_sentido_solo_se_envia_cuando_cambia_y_antes_que_la_velocidad():
    logic = lista_para_andar()
    logic.on_drive_command(0.5, 0.0, now=10.0)
    assert control(logic, now=10.0)[0] == 'C:SET_DIR:1'
    logic.on_drive_command(-0.5, 0.0, now=10.1)
    assert control(logic, now=10.1) == ['C:SET_DIR:0', 'C:SET_STEER:105', 'C:SET_SPEED:255']
    logic.on_drive_command(-0.5, 0.0, now=10.2)
    assert 'C:SET_DIR:0' not in lineas(logic, now=10.2)


def test_parar_no_cambia_el_sentido():
    logic = lista_para_andar()
    logic.on_drive_command(-0.5, 0.0, now=10.0)
    lineas(logic, now=10.0)
    logic.on_drive_command(0.0, 0.0, now=10.1)
    assert control(logic, now=10.1) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


def test_sin_ningun_comando_se_envia_velocidad_cero_y_direccion_al_centro():
    assert control(lista_para_andar(), now=0.0) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


def test_un_comando_viejo_se_descarta_y_se_frena():
    logic = lista_para_andar(cmd_timeout_s=0.25)
    logic.on_drive_command(0.5, math.radians(30.0), now=10.0)
    assert 'C:SET_SPEED:255' in lineas(logic, now=10.25)
    assert control(logic, now=10.5) == ['C:SET_STEER:105', 'C:SET_SPEED:0']


# --- latido y enlace ------------------------------------------------------------------------

def test_andando_sin_que_el_esp32_hable_el_latido_mantiene_el_enlace():
    # El caso que fallaba: a velocidad constante el firmware no emite nada. El latido
    # (el mismo stop vigente) hace que conteste CMD_EXECUTED y el enlace no se cae.
    logic = lista_para_andar()
    caidas = []
    for paso in range(100):  # 5 s a 20 Hz
        now = 1.0 + paso * 0.05
        logic.on_drive_command(0.3, 0.0, now)
        for linea in gestion(logic, now):
            comando = linea.split(':')[1]
            recibir(logic, now + 0.02, 'EVENT:CMD_EXECUTED:%s' % comando)
        if not logic.state(now + 0.04).link_up:
            caidas.append(now)
    assert caidas == []


def test_el_latido_reenvia_el_stop_vigente_cada_periodo():
    logic = lista_para_andar(heartbeat_period_s=0.3)
    assert gestion(logic, now=0.0) == ['M:SYS_RESUME:0']
    assert gestion(logic, now=0.1) == []
    assert gestion(logic, now=0.3) == ['M:SYS_RESUME:0']


def test_con_el_auto_detenido_el_latido_repite_el_stop_y_nunca_lo_suelta():
    logic = lista_para_andar()
    logic.request_stop(True)
    enviados = []
    for paso in range(40):
        now = 1.0 + paso * 0.05
        enviados += gestion(logic, now)
        recibir(logic, now, 'EVENT:CMD_EXECUTED:SYS_STOP')
    assert enviados and set(enviados) == {'M:SYS_STOP:0'}


def test_antes_de_recibir_algo_del_esp32_el_enlace_esta_caido():
    estado = logica().state(now=0.0)
    assert estado.link_up is False
    assert estado.esp32_state == ''


def test_un_evento_levanta_el_enlace():
    logic = logica()
    recibir(logic, 1.0, 'EVENT:CMD_EXECUTED:SYS_RESUME')
    assert logic.state(now=1.5).link_up is True


def test_sin_eventos_por_mas_del_tiempo_limite_el_enlace_se_da_por_caido():
    logic = logica(link_timeout_s=1.0)
    recibir(logic, 1.0, 'EVENT:CMD_EXECUTED:SYS_RESUME')
    assert logic.state(now=2.0).link_up is True
    assert logic.state(now=2.05).link_up is False


@pytest.mark.parametrize('linea', ['[Control] started', 'ruido', ''])
def test_la_depuracion_y_el_ruido_no_cuentan_como_senal_de_vida(linea):
    logic = logica()
    recibir(logic, 1.0, linea)
    assert logic.state(now=1.0).link_up is False


def test_al_reabrir_el_puerto_se_olvida_lo_que_se_sabia_del_esp32():
    logic = lista_para_andar()
    recibir(logic, 0.0, 'EVENT:OBSTACLE_DETECTED:22', 'EVENT:ULTRASONIC_SILENT')
    logic.on_port_opened()
    estado = logic.state(now=0.0)
    assert (estado.link_up, estado.esp32_state, estado.obstacle) == (False, '', False)


# --- armado ---------------------------------------------------------------------------------

def test_al_abrir_el_puerto_se_arma_sin_configurar_ningun_modo():
    logic = logica()
    logic.on_port_opened()
    assert gestion(logic, now=0.0) == ['M:SYS_ARM:0']
    assert not any('SYS_MODE' in linea for linea in lineas(logic, now=1.0))


def test_se_arma_y_se_reintenta_hasta_la_confirmacion():
    logic = logica(arm_retry_s=0.5)
    logic.on_port_opened()
    assert gestion(logic, now=0.0) == ['M:SYS_ARM:0']
    assert 'M:SYS_ARM:0' not in lineas(logic, now=0.25)
    assert gestion(logic, now=0.5) == ['M:SYS_ARM:0']
    recibir(logic, 0.55, 'EVENT:STATE_CHANGED:RUNNING', 'EVENT:CMD_EXECUTED:SYS_ARM')
    for paso in range(20):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=1.0 + paso * 0.05)


def test_si_el_esp32_ya_estaba_armado_basta_la_confirmacion():
    # Ya armado, el firmware confirma SYS_ARM sin emitir STATE_CHANGED.
    logic = logica()
    logic.on_port_opened()
    lineas(logic, now=0.0)
    recibir(logic, 0.05, 'EVENT:CMD_EXECUTED:SYS_ARM')
    for paso in range(20):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=0.5 + paso * 0.05)


def test_nunca_se_envian_dos_comandos_de_gestion_en_el_mismo_ciclo():
    logic = logica()
    logic.on_port_opened()
    logic.request_stop(True)
    logic.request_obstacle_reset()
    for paso in range(40):
        now = paso * 0.05
        assert len(gestion(logic, now)) <= 1
        if paso == 3:
            recibir(logic, now, 'EVENT:CMD_REJECTED:SYS_ARM:ESTOP_ACTIVE')


def test_si_el_esp32_ya_estaba_andando_no_se_lo_arma_de_nuevo():
    logic = logica()
    logic.on_port_opened()
    recibir(logic, 0.0, 'EVENT:STATE_CHANGED:RUNNING')
    assert 'M:SYS_ARM:0' not in lineas(logic, now=0.0)


def test_un_fault_posterior_no_se_rearma_solo():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:WATCHDOG_TIMEOUT', 'EVENT:STATE_CHANGED:FAULT')
    for paso in range(40):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=1.0 + paso * 0.05)
    assert logic.state(now=1.0).fault is True


@pytest.mark.parametrize('evento', [
    'EVENT:WATCHDOG_TIMEOUT',
    'EVENT:ESTOP_TRIGGERED:GPIO',
    'EVENT:STATE_CHANGED:FAULT',
])
def test_cada_motivo_de_fault_se_refleja_en_el_estado(evento):
    logic = lista_para_andar()
    recibir(logic, 1.0, evento)
    assert logic.state(now=1.0).fault is True


def test_en_fault_el_latido_sigue_y_el_enlace_no_se_cae():
    # SYS_STOP y SYS_RESUME se confirman en cualquier estado, también en FAULT.
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_CHANGED:FAULT')
    assert gestion(logic, now=1.0) == ['M:SYS_RESUME:0']


def test_pedir_armado_saca_al_esp32_del_fault():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:STATE_CHANGED:FAULT')
    logic.request_arm()
    assert gestion(logic, now=1.1) == ['M:SYS_ARM:0']
    recibir(logic, 1.12, 'EVENT:STATE_CHANGED:RUNNING', 'EVENT:CMD_EXECUTED:SYS_ARM')
    assert logic.state(now=1.2).fault is False
    for paso in range(20):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=1.2 + paso * 0.05)


def test_un_armado_rechazado_por_el_e_stop_se_sigue_reintentando():
    logic = logica(arm_retry_s=0.5)
    logic.on_port_opened()
    lineas(logic, now=0.0)
    recibir(logic, 0.1, 'EVENT:CMD_REJECTED:SYS_ARM:ESTOP_ACTIVE')
    assert gestion(logic, now=0.5) == ['M:SYS_ARM:0']


def test_si_el_esp32_se_reinicia_se_vuelve_a_armar():
    logic = lista_para_andar()
    logic.on_drive_command(-0.5, 0.0, now=1.0)
    lineas(logic, now=1.0)
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    assert gestion(logic, now=2.0) == ['M:SYS_ARM:0']


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
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22', 'EVENT:STATE_CHANGED:STOPPED')
    assert logic.state(now=1.0).esp32_state == 'STOPPED'
    assert logic.state(now=1.0).fault is False
    assert 'M:SYS_ARM:0' not in lineas(logic, now=1.0)


def test_si_el_esp32_arranca_con_un_obstaculo_queda_armado_en_stopped():
    logic = logica()
    logic.on_port_opened()
    recibir(logic, 0.0, *ARRANQUE_DEL_ESP32, 'EVENT:OBSTACLE_DETECTED:15')
    assert gestion(logic, now=0.0) == ['M:SYS_ARM:0']
    recibir(logic, 0.05, 'EVENT:STATE_CHANGED:STOPPED', 'EVENT:CMD_EXECUTED:SYS_ARM')
    assert logic.state(now=0.05).esp32_state == 'STOPPED'
    for paso in range(20):
        assert 'M:SYS_ARM:0' not in lineas(logic, now=0.1 + paso * 0.05)


# --- detener y arrancar (STOPPED) -------------------------------------------------------------

def test_sin_pedidos_el_latido_manda_resume_que_no_cambia_nada():
    logic = lista_para_andar()
    assert gestion(logic, now=1.0) == ['M:SYS_RESUME:0']
    assert logic.stop_requested() is None


def test_detener_envia_sys_stop_y_lo_reintenta_hasta_la_confirmacion():
    logic = lista_para_andar(arm_retry_s=0.5)
    logic.request_stop(True)
    assert gestion(logic, now=1.0) == ['M:SYS_STOP:0']
    assert gestion(logic, now=1.25) == []
    assert gestion(logic, now=1.5) == ['M:SYS_STOP:0']
    recibir(logic, 1.55, 'EVENT:STATE_CHANGED:STOPPED', 'EVENT:CMD_EXECUTED:SYS_STOP')
    assert logic.state(now=1.6).stopped is True
    # Confirmado: ya no hay reintentos, solo el latido cada heartbeat_period_s.
    assert gestion(logic, now=1.6) == []
    assert gestion(logic, now=1.8) == ['M:SYS_STOP:0']


def test_arrancar_envia_sys_resume_hasta_la_confirmacion():
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_EXECUTED:SYS_STOP')
    logic.request_stop(False)
    assert gestion(logic, now=1.1) == ['M:SYS_RESUME:0']
    recibir(logic, 1.15, 'EVENT:CMD_EXECUTED:SYS_RESUME', 'EVENT:STATE_CHANGED:RUNNING')
    assert gestion(logic, now=1.2) == []
    assert logic.state(now=1.2).esp32_state == 'RUNNING'


def test_una_confirmacion_del_pedido_anterior_no_cuenta():
    # Se pidió detener y enseguida arrancar: el CMD_EXECUTED:SYS_STOP que llega
    # tarde no confirma el arranque.
    logic = lista_para_andar(arm_retry_s=0.5)
    logic.request_stop(True)
    lineas(logic, now=1.0)
    logic.request_stop(False)
    lineas(logic, now=1.05)
    recibir(logic, 1.1, 'EVENT:CMD_EXECUTED:SYS_STOP')
    assert gestion(logic, now=1.6) == ['M:SYS_RESUME:0']


def test_repetir_el_mismo_pedido_no_lo_vuelve_a_poner_pendiente():
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_EXECUTED:SYS_STOP')
    logic.request_stop(True)
    assert gestion(logic, now=1.1) == []


def test_si_el_esp32_se_reinicia_se_le_vuelve_a_enviar_el_stop_antes_de_armar():
    # El ESP32 olvida el SYS_STOP al reiniciarse.
    logic = lista_para_andar()
    logic.request_stop(True)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_EXECUTED:SYS_STOP')
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    enviados = gestion(logic, now=2.0) + gestion(logic, now=2.05)
    assert enviados == ['M:SYS_STOP:0', 'M:SYS_ARM:0']


def test_tras_un_reinicio_sin_stop_pedido_primero_se_arma():
    logic = lista_para_andar()
    logic.request_stop(False)
    lineas(logic, now=1.0)
    recibir(logic, 1.05, 'EVENT:CMD_EXECUTED:SYS_RESUME')
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    assert gestion(logic, now=2.0) == ['M:SYS_ARM:0']


def test_al_cerrar_se_frena_y_se_desarma():
    assert logica().shutdown_frames() == b'C:SET_SPEED:0\nM:SYS_DISARM:0\n'


# --- obstáculo del ESP32 ------------------------------------------------------------------

def frenado_por_obstaculo(**cambios):
    """Un puente andando que acaba de recibir un obstáculo a 22 cm en t=1."""
    logic = lista_para_andar(**cambios)
    logic.on_drive_command(0.3, 0.0, now=1.0)
    lineas(logic, now=1.0)
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:22', 'EVENT:STATE_CHANGED:STOPPED')
    return logic


def resets(logic, desde, hasta):
    enviados = []
    paso = 0
    while desde + paso * 0.05 <= hasta:
        enviados += [g for g in gestion(logic, desde + paso * 0.05) if 'OBSTACLE_RESET' in g]
        paso += 1
    return enviados


def test_un_obstaculo_del_esp32_se_informa_con_su_distancia():
    logic = frenado_por_obstaculo()
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.obstacle_cm, estado.esp32_state) == (True, 22, 'STOPPED')


def test_al_despejarse_el_obstaculo_ya_no_se_informa():
    logic = frenado_por_obstaculo()
    recibir(logic, 2.0, 'EVENT:OBSTACLE_CLEARED', 'EVENT:STATE_CHANGED:RUNNING')
    estado = logic.state(now=2.0)
    assert (estado.obstacle, estado.obstacle_cm, estado.esp32_state) == (False, None, 'RUNNING')


def test_una_distancia_ilegible_se_informa_como_desconocida():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:OBSTACLE_DETECTED:abc')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.obstacle_cm) == (True, None)


def test_un_stopped_pedido_por_servicio_no_es_un_obstaculo():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:CMD_EXECUTED:SYS_STOP', 'EVENT:STATE_CHANGED:STOPPED')
    estado = logic.state(now=1.0)
    assert (estado.obstacle, estado.esp32_state) == (False, 'STOPPED')


@pytest.mark.parametrize('evento', [
    'EVENT:STATE_CHANGED:FAULT', 'EVENT:STATE_CHANGED:DISARMED', 'EVENT:WATCHDOG_TIMEOUT',
])
def test_el_obstaculo_sobrevive_a_un_cambio_de_estado(evento):
    # En el firmware el bloqueo por obstáculo es independiente del estado: al
    # rearmar, el ESP32 vuelve a STOPPED si el obstáculo sigue.
    logic = frenado_por_obstaculo()
    recibir(logic, 2.0, evento)
    assert logic.state(now=2.0).obstacle is True


def test_si_el_esp32_se_reinicia_se_olvida_el_obstaculo():
    logic = frenado_por_obstaculo()
    recibir(logic, 2.0, *ARRANQUE_DEL_ESP32)
    assert logic.state(now=2.0).obstacle is False


def test_con_un_obstaculo_el_puente_no_manda_ningun_freno_y_sigue_con_el_comando():
    # Frena el firmware. El puente sigue enviando el comando vigente: apenas el ESP32
    # suelte el bloqueo, el auto vuelve a andar con el próximo ciclo.
    logic = frenado_por_obstaculo()
    logic.on_drive_command(0.5, 0.0, now=1.05)
    enviadas = lineas(logic, now=1.05) + lineas(logic, now=1.1)
    assert 'E:BRAKE_NOW:0' not in enviadas
    assert 'C:SET_SPEED:255' in enviadas


def test_el_silencio_del_sensor_se_informa():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:ULTRASONIC_SILENT')
    assert logic.state(now=1.0).ultrasonic_silent is True
    recibir(logic, 2.0, 'EVENT:ULTRASONIC_OK')
    assert logic.state(now=2.0).ultrasonic_silent is False


# --- destrabe del obstáculo ---------------------------------------------------------------

def test_frenado_y_con_el_sensor_en_silencio_se_destraba_solo():
    logic = frenado_por_obstaculo()
    assert resets(logic, 1.05, 3.0) == []
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 3.05, 3.2) == ['M:OBSTACLE_RESET:0']


def test_el_destrabe_automatico_se_reintenta_hasta_la_confirmacion_y_despues_no_se_repite():
    logic = frenado_por_obstaculo(arm_retry_s=0.5)
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    assert len(resets(logic, 3.05, 3.6)) == 2
    recibir(logic, 3.62, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED',
            'EVENT:STATE_CHANGED:RUNNING')
    assert resets(logic, 3.65, 6.0) == []
    assert logic.state(now=3.65).obstacle is False


def test_sin_silencio_no_se_destraba_solo():
    # Con lecturas válidas el obstáculo sigue ahí (o hay algo al alcance): decide el firmware.
    logic = frenado_por_obstaculo()
    assert resets(logic, 1.05, 10.0) == []


def test_el_silencio_sin_obstaculo_no_manda_nada():
    logic = lista_para_andar()
    recibir(logic, 1.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 1.05, 3.0) == []


def test_si_vuelve_a_frenar_enseguida_no_se_insiste():
    logic = frenado_por_obstaculo()
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 3.05, 3.05) == ['M:OBSTACLE_RESET:0']
    recibir(logic, 3.1, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED')
    # El obstáculo seguía ahí: el ESP32 vuelve a bloquear y después se queda sin lecturas.
    recibir(logic, 3.3, 'EVENT:OBSTACLE_DETECTED:8')
    recibir(logic, 5.5, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 5.55, 10.0) == []
    assert logic.state(now=10.0).obstacle is True


def test_un_destrabe_a_mano_rehabilita_el_automatico():
    logic = frenado_por_obstaculo()
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    resets(logic, 3.05, 3.05)
    recibir(logic, 3.1, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED')
    recibir(logic, 3.3, 'EVENT:OBSTACLE_DETECTED:8', 'EVENT:ULTRASONIC_SILENT')
    logic.request_obstacle_reset()
    assert resets(logic, 3.35, 3.35) == ['M:OBSTACLE_RESET:0']
    recibir(logic, 3.4, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED')
    # Una frenada nueva, mucho después, vuelve a destrabarse sola.
    recibir(logic, 20.0, 'EVENT:OBSTACLE_DETECTED:20')
    recibir(logic, 22.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 22.05, 22.1) == ['M:OBSTACLE_RESET:0']


def test_una_frenada_nueva_despues_de_un_rato_se_destraba_otra_vez():
    logic = frenado_por_obstaculo()
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    resets(logic, 3.05, 3.05)
    recibir(logic, 3.1, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED')
    recibir(logic, 30.0, 'EVENT:OBSTACLE_DETECTED:20')
    recibir(logic, 32.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 32.05, 32.1) == ['M:OBSTACLE_RESET:0']


def test_un_despeje_del_propio_firmware_rehabilita_el_automatico():
    logic = frenado_por_obstaculo()
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    resets(logic, 3.05, 3.05)
    recibir(logic, 3.1, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET', 'EVENT:OBSTACLE_CLEARED')
    recibir(logic, 3.3, 'EVENT:OBSTACLE_DETECTED:8')
    # Lecturas lejanas válidas: suelta el propio firmware.
    recibir(logic, 6.0, 'EVENT:OBSTACLE_CLEARED')
    recibir(logic, 7.0, 'EVENT:OBSTACLE_DETECTED:20')
    recibir(logic, 9.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 9.05, 9.1) == ['M:OBSTACLE_RESET:0']


def test_con_el_destrabe_automatico_apagado_solo_se_destraba_a_mano():
    logic = frenado_por_obstaculo(auto_obstacle_reset=False)
    recibir(logic, 3.0, 'EVENT:ULTRASONIC_SILENT')
    assert resets(logic, 3.05, 10.0) == []
    logic.request_obstacle_reset()
    assert resets(logic, 10.05, 10.05) == ['M:OBSTACLE_RESET:0']


def test_el_destrabe_a_mano_se_reintenta_hasta_la_confirmacion():
    logic = frenado_por_obstaculo(arm_retry_s=0.5)
    logic.request_obstacle_reset()
    assert len(resets(logic, 1.05, 1.6)) == 2
    recibir(logic, 1.62, 'EVENT:CMD_EXECUTED:OBSTACLE_RESET')
    assert resets(logic, 1.65, 3.0) == []


# --- configuración ---------------------------------------------------------------------------

@pytest.mark.parametrize('cambios', [
    {'link_timeout_s': 0.0},
    {'cmd_timeout_s': -1.0},
    {'arm_retry_s': 0.0},
    {'heartbeat_period_s': 0.0},
    {'obstacle_reblock_window_s': 0.0},
    {'heartbeat_period_s': 1.0, 'link_timeout_s': 1.0},
])
def test_una_configuracion_invalida_se_rechaza(cambios):
    with pytest.raises(ValueError):
        logica(**cambios)
