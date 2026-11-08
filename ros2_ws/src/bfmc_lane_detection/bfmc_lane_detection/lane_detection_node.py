"""Nodo ROS2 de bfmc_lane_detection: convierte las imágenes de la cámara en LaneState."""
import dataclasses
import threading

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.msg import HardwareStatus, LaneState
from bfmc_lane_detection.ground_calibration import (
    calibrar_piso, CalibrationError, CAMPOS_CALCULADOS, GroundCalibrationConfig, GroundView,
    Intrinsics, lane_config_desde_parametros, medir_pose, PARAMETROS_DE_GEOMETRIA,
)
from bfmc_lane_detection.image_utils import imagen_a_bgr, profundidad_a_metros
from bfmc_lane_detection.lane_debug import dibujar_camara, dibujar_cenital
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
from bfmc_lane_detection.lane_kalman import LaneKalman
from bfmc_lane_detection.lane_tracker import LaneTracker, NOMBRES_DE_MODO
from bfmc_lane_detection.param_store import guardar_parametros
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

PARAMETROS_TEXTO = (
    'image_topic', 'lane_topic', 'debug_overlay_topic', 'debug_birdview_topic',
    'depth_topic', 'depth_info_topic', 'hardware_status_topic', 'calibrate_service',
    'save_service', 'persist_file', 'cmd_drive_topic')
PARAMETROS_ENTEROS = (
    'morph_kernel_px', 'warp_width', 'warp_height', 'n_windows', 'window_margin_px',
    'min_pixels', 'full_confidence_pixels', 'intersection_min_rows', 'min_windows',
)
PARAMETROS_DECIMALES = (
    'roi_top_fraction', 'lane_width_m',
    'reference_row_fraction', 'intersection_band_top_fraction', 'intersection_row_fraction',
    'debug_max_fps', 'max_residual_m', 'min_curve_radius_m', 'lane_width_tolerance_m',
    'lost_timeout_s', 'search_margin_m', 'kalman_max_std_m', 'kalman_gate_chi2',
)
PARAMETROS_ARREGLOS = ('white_hsv_min', 'white_hsv_max')

# Calibración con la profundidad (ver ground_calibration.py): nombre del parámetro -> campo
# de GroundCalibrationConfig. Se pueden cambiar en vivo; un valor inválido se rechaza.
PARAMETROS_DE_CALIBRACION = {
    'calib_depth_min_m': 'depth_min_m',
    'calib_depth_max_m': 'depth_max_m',
    'calib_subsample': 'subsample',
    'calib_min_points': 'min_points',
    'calib_ransac_iters': 'ransac_iters',
    'calib_ransac_seed': 'ransac_seed',
    'calib_inlier_m': 'inlier_m',
    'calib_min_inlier_fraction': 'min_inlier_fraction',
    'calib_height_range_m': 'height_range_m',
    'calib_pitch_range_deg': 'pitch_range_deg',
}
CALIB_ENTEROS = (
    'calib_subsample', 'calib_min_points', 'calib_ransac_iters', 'calib_ransac_seed',
    'calib_frames', 'calib_check_frames')
CALIB_ARREGLOS = ('calib_height_range_m', 'calib_pitch_range_deg')
CALIB_DECIMALES = tuple(
    nombre for nombre in PARAMETROS_DE_CALIBRACION
    if nombre not in CALIB_ENTEROS + CALIB_ARREGLOS) + (
    'calib_timeout_s', 'calib_check_height_tol_m', 'calib_check_pitch_tol_deg')

# Parámetros con los que se arma el detector: los de LaneConfig que pasan tal cual, más los
# de la geometría (pose, intrínsecos y rectángulo), de los que salen warp_src y la escala.
PARAMETROS_DEL_DETECTOR = tuple(
    campo.name for campo in dataclasses.fields(LaneConfig)
    if campo.name not in CAMPOS_CALCULADOS) + PARAMETROS_DE_GEOMETRIA

# Lo que guarda el botón Guardar: las medidas y lo que se ajusta en el visor. Nada más.
PARAMETROS_PERSISTENTES = (
    'lane_width_m',) + PARAMETROS_DE_GEOMETRIA + (
    'white_hsv_min', 'white_hsv_max', 'roi_top_fraction')

# Filtro de Kalman del carril (A2-154): se rearma con cualquiera de estos cambios.
PARAMETROS_DEL_FILTRO = (
    'kalman_enabled', 'kalman_q', 'kalman_r_two_lines', 'kalman_r_one_line',
    'kalman_max_std_m', 'kalman_gate_chi2')

# Estado del ESP32 en el que el auto se mueve (bfmc_interfaces/HardwareStatus).
ESTADO_EN_MARCHA = 'RUNNING'

# Cada cuánto se mira si terminó el chequeo de la cámara al arrancar.
PERIODO_CHEQUEO_S = 0.2


class _Captura:
    """Los cuadros de profundidad que se juntan para una calibración o un chequeo."""

    def __init__(self, cuadros):
        self.cuadros = cuadros
        self.profundidades = []
        self.intrinsecos = None
        self.error = None
        self.lista = threading.Event()

    def completa(self):
        return len(self.profundidades) >= self.cuadros and self.intrinsecos is not None


# Los topics solo se eligen al arrancar: cambiarlos en vivo obligaría a rehacer las
# suscripciones y publicaciones. Todo lo demás se puede cambiar con el nodo andando.
PARAMETROS_SOLO_AL_ARRANCAR = PARAMETROS_TEXTO


class LaneDetectionNode(Node):
    """Lee las imágenes de color y publica el estado del carril en /perception/lane.

    Cada imagen se procesa sola. El encabezado del resultado es el de la imagen, así
    quien lo consume sabe de qué instante es la medición.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.

    Los parámetros de la visión se pueden cambiar con el nodo andando (por ejemplo con
    `ros2 param set` o desde el visor de calibración). El cambio se valida antes de
    aplicarse: un valor inválido se rechaza y el detector sigue como estaba. No se guarda
    en el YAML. Un cambio arma un detector nuevo y lo reemplaza de una vez; la imagen que
    se está procesando termina con el detector con el que empezó.

    Si hay alguien suscripto a las imágenes de depuración, además publica lo que ve el
    detector (ver lane_debug.py), a lo sumo `debug_max_fps` veces por segundo.

    Geometría: `warp_src` y la escala NO son parámetros. Se calculan con la pose de la
    cámara (`calib_height_m`, `calib_pitch_deg`, `calib_roll_deg`), sus intrínsecos
    (`camera_intrinsics`) y el rectángulo del piso (`calib_forward_near_m`,
    `calib_forward_far_m`, `calib_half_width_m`). Así la escala queda atada al trapecio y
    cambiar el ancho del carril (`lane_width_m`, dato de la pista) no la toca.

    Calibración (`calibrate_service`, std_srvs/Trigger): se suscribe a la profundidad
    alineada al color solo mientras calibra, junta `calib_frames` cuadros, mide la pose
    (ver ground_calibration.py) y la aplica junto con los intrínsecos. Si falla, no cambia
    nada y responde el motivo. Al arrancar, si `calib_check_on_start`, compara la cámara
    con la última calibración y avisa si se movió.

    Con el auto andando (el ESP32 en RUNNING según `hardware_status_topic`) se rechazan la
    calibración y los cambios de geometría: cambiarían la escala en plena marcha. Sin
    estado del ESP32 (por ejemplo, sin el puente lanzado) el auto no se puede mover y se
    permiten.

    Guardar (`save_service`, std_srvs/Trigger): escribe PARAMETROS_PERSISTENTES en
    `persist_file`, fuera del repo; los launch lo cargan después del params.yaml.

    Hilos: el servicio espera los cuadros de profundidad, así que la profundidad y la
    calibración tienen sus propios grupos de callbacks y el nodo corre con un
    MultiThreadedExecutor. Las imágenes de color y los cambios de parámetros siguen en el
    grupo por defecto, de a uno.
    """

    def __init__(self, **opciones):
        # `opciones` pasa a Node (por ejemplo parameter_overrides, para los tests).
        super().__init__('lane_detection', **opciones)

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_ENTEROS:
            self.declare_parameter(nombre, Parameter.Type.INTEGER)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('publish_debug', Parameter.Type.BOOL)
        self.declare_parameter('white_hsv_min', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('white_hsv_max', Parameter.Type.INTEGER_ARRAY)
        for nombre in PARAMETROS_DE_GEOMETRIA:
            tipo = (Parameter.Type.DOUBLE_ARRAY if nombre == 'camera_intrinsics'
                    else Parameter.Type.DOUBLE)
            self.declare_parameter(nombre, tipo)
        for nombre in CALIB_ENTEROS:
            self.declare_parameter(nombre, Parameter.Type.INTEGER)
        for nombre in CALIB_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        for nombre in CALIB_ARREGLOS:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('calib_check_on_start', Parameter.Type.BOOL)
        self.declare_parameter('width_check_fractions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('kalman_enabled', Parameter.Type.BOOL)
        for nombre in ('kalman_q', 'kalman_r_two_lines', 'kalman_r_one_line'):
            self.declare_parameter(nombre, Parameter.Type.DOUBLE_ARRAY)
        self._config_calibracion()   # falla al arrancar si la calibración está mal configurada

        self._detector = LaneDetector(self._configuracion())
        # La memoria del carril vive aparte del detector: rehacer el detector por un cambio
        # de parámetros no la borra.
        self._tracker = LaneTracker(self._valor('lost_timeout_s'))
        self._configurar_filtro()   # falla al arrancar si los parámetros del filtro son inválidos
        # Velocidad del auto para predecir: la comandada (/cmd/drive). No hay odometría.
        # Sin comandos, 0: el filtro mantiene el estado y su incertidumbre crece.
        self._velocidad_mps = 0.0

        # Las imágenes se leen con calidad "best effort" y cola de 1: si el nodo se
        # atrasa, conviene procesar la imagen más nueva y no acumular retraso.
        imagenes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        resultados = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Las de depuración son "reliable": las acepta tanto un suscriptor "reliable" como
        # uno "best effort".
        depuracion = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            LaneState, self._valor('lane_topic'), resultados)
        self._publicador_camara = self.create_publisher(
            Image, self._valor('debug_overlay_topic'), depuracion)
        self._publicador_cenital = self.create_publisher(
            Image, self._valor('debug_birdview_topic'), depuracion)
        self._subscription = self.create_subscription(
            Image, self._valor('image_topic'), self._on_image, imagenes)
        self._ultima_depuracion = None

        self._candado = threading.Lock()
        self._captura = None
        self._suscripciones_profundidad = []
        self._grupo_profundidad = MutuallyExclusiveCallbackGroup()
        self._grupo_calibracion = MutuallyExclusiveCallbackGroup()
        self._servicio_calibrar = self.create_service(
            Trigger, self._valor('calibrate_service'), self._al_calibrar,
            callback_group=self._grupo_calibracion)
        self._servicio_guardar = self.create_service(
            Trigger, self._valor('save_service'), self._al_guardar,
            callback_group=self._grupo_calibracion)
        self._estado_esp32 = ''
        self._suscripcion_comando = self.create_subscription(
            AckermannDriveStamped, self._valor('cmd_drive_topic'), self._on_comando,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._suscripcion_estado = self.create_subscription(
            HardwareStatus, self._valor('hardware_status_topic'), self._on_estado,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._chequeo = None
        if self._valor('calib_check_on_start'):
            self._empezar_chequeo()

        self.add_on_set_parameters_callback(self._al_cambiar_parametros)
        self.get_logger().info('lane_detection iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _configuracion(self, cambios=None):
        """Arma la configuración del detector con los parámetros, más los `cambios` dados."""
        cambios = cambios or {}
        return lane_config_desde_parametros({
            nombre: cambios.get(nombre, self._valor(nombre))
            for nombre in PARAMETROS_DEL_DETECTOR})

    def _configurar_filtro(self, cambios=None):
        """Arma el filtro con los parámetros (más `cambios`) y se lo da al tracker."""
        cambios = cambios or {}
        valor = {n: cambios.get(n, self._valor(n)) for n in PARAMETROS_DEL_FILTRO}
        filtro = None
        if valor['kalman_enabled']:
            filtro = LaneKalman(valor['kalman_q'], valor['kalman_r_two_lines'],
                                valor['kalman_r_one_line'], valor['kalman_gate_chi2'])
        self._tracker.usar_kalman(filtro, valor['kalman_max_std_m'])

    def _on_comando(self, msg: AckermannDriveStamped):
        self._velocidad_mps = float(msg.drive.speed)

    def _on_estado(self, msg: HardwareStatus):
        self._estado_esp32 = msg.esp32_state

    def _auto_en_marcha(self):
        return self._estado_esp32 == ESTADO_EN_MARCHA

    def _al_cambiar_parametros(self, parametros):
        """Valida un cambio de parámetros y, si es válido, rehace el detector."""
        cambios = {parametro.name: parametro.value for parametro in parametros}

        fijos = sorted(nombre for nombre in cambios if nombre in PARAMETROS_SOLO_AL_ARRANCAR)
        if fijos:
            return SetParametersResult(
                successful=False,
                reason='%s solo se puede cambiar al arrancar el nodo' % ', '.join(fijos))
        geometria = sorted(nombre for nombre in cambios if nombre in PARAMETROS_DE_GEOMETRIA)
        if geometria and self._auto_en_marcha():
            return SetParametersResult(
                successful=False,
                reason='el auto está andando: frenalo antes de cambiar %s' % ', '.join(geometria))
        if 'lost_timeout_s' in cambios:
            try:
                self._tracker.lost_timeout_s = cambios['lost_timeout_s']
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        if any(nombre in PARAMETROS_DEL_FILTRO for nombre in cambios):
            try:
                self._configurar_filtro(cambios)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        if 'debug_max_fps' in cambios and not cambios['debug_max_fps'] > 0.0:
            return SetParametersResult(
                successful=False, reason='debug_max_fps debe ser mayor que 0')
        if any(nombre in PARAMETROS_DE_CALIBRACION for nombre in cambios):
            try:
                self._config_calibracion(cambios)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        positivos = ('calib_frames', 'calib_check_frames', 'calib_timeout_s')
        malos = sorted(nombre for nombre in positivos
                       if nombre in cambios and not cambios[nombre] > 0)
        if malos:
            return SetParametersResult(
                successful=False, reason='%s debe ser mayor que 0' % ', '.join(malos))

        de_la_vision = sorted(nombre for nombre in cambios if nombre in PARAMETROS_DEL_DETECTOR)
        if de_la_vision:
            try:
                configuracion = self._configuracion(cambios)
                detector = LaneDetector(configuracion)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            self._detector = detector
            # Los polinomios previos están en píxeles de la vista vieja: se buscan de nuevo.
            self._tracker.olvidar_previas()
            self.get_logger().info(
                'detector actualizado: %s' % ', '.join(de_la_vision), throttle_duration_sec=1.0)
        return SetParametersResult(successful=True)

    # --- calibración -------------------------------------------------------------------
    def _config_calibracion(self, cambios=None):
        cambios = cambios or {}
        valores = {}
        for nombre, campo in PARAMETROS_DE_CALIBRACION.items():
            valor = cambios.get(nombre, self._valor(nombre))
            valores[campo] = tuple(valor) if nombre in CALIB_ARREGLOS else valor
        return GroundCalibrationConfig(**valores)

    def _empezar_captura(self, cuadros):
        """Empieza a juntar cuadros de profundidad. Reemplaza a una captura en curso."""
        captura = _Captura(cuadros)
        with self._candado:
            self._captura = captura
            if not self._suscripciones_profundidad:
                # "Best effort" y cola de 1: lo acepta cualquier publicador y no acumula.
                calidad = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                self._suscripciones_profundidad = [
                    self.create_subscription(
                        Image, self._valor('depth_topic'), self._on_depth, calidad,
                        callback_group=self._grupo_profundidad),
                    self.create_subscription(
                        CameraInfo, self._valor('depth_info_topic'), self._on_depth_info,
                        calidad, callback_group=self._grupo_profundidad),
                ]
        return captura

    def _terminar_captura(self, captura):
        """Deja de escuchar la profundidad, si `captura` sigue siendo la captura en curso."""
        with self._candado:
            if self._captura is not captura:
                return
            self._captura = None
            suscripciones, self._suscripciones_profundidad = self._suscripciones_profundidad, []
        for suscripcion in suscripciones:
            self.destroy_subscription(suscripcion)

    def _on_depth(self, msg: Image):
        with self._candado:
            captura = self._captura
        if captura is None or len(captura.profundidades) >= captura.cuadros:
            return
        try:
            metros = profundidad_a_metros(
                msg.height, msg.width, msg.encoding, msg.step, msg.data, bool(msg.is_bigendian))
        except ValueError as error:
            captura.error = 'la profundidad de %s no se puede leer: %s' % (
                self._valor('depth_topic'), error)
            captura.lista.set()
            return
        captura.profundidades.append(metros)
        if captura.completa():
            captura.lista.set()

    def _on_depth_info(self, msg: CameraInfo):
        with self._candado:
            captura = self._captura
        if captura is None or captura.intrinsecos is not None:
            return
        fx, fy = msg.k[0], msg.k[4]
        if fx <= 0.0 or fy <= 0.0 or msg.width == 0 or msg.height == 0:
            captura.error = '%s no trae intrínsecos válidos' % self._valor('depth_info_topic')
            captura.lista.set()
            return
        captura.intrinsecos = Intrinsics(fx, fy, msg.k[2], msg.k[5], msg.width, msg.height)
        if captura.completa():
            captura.lista.set()

    def _motivo_sin_datos(self, captura):
        return ('en %.0f s llegaron %d de %d imágenes de profundidad%s: revisá que la cámara '
                'esté andando con align_depth activado y publique en %s y %s'
                % (self._valor('calib_timeout_s'), len(captura.profundidades), captura.cuadros,
                   '' if captura.intrinsecos else ' y ningún camera_info',
                   self._valor('depth_topic'), self._valor('depth_info_topic')))

    def _al_calibrar(self, _pedido, respuesta):
        """Calibra la vista cenital con la profundidad y aplica el resultado en vivo."""
        respuesta.success = False
        if self._auto_en_marcha():
            respuesta.message = 'calibración rechazada: el auto está andando. Frenalo y calibrá'
            self.get_logger().warning(respuesta.message)
            return respuesta
        configuracion = self._config_calibracion()
        vista = GroundView(self._valor('calib_forward_near_m'),
                           self._valor('calib_forward_far_m'), self._valor('calib_half_width_m'))
        captura = self._empezar_captura(self._valor('calib_frames'))
        try:
            llego = captura.lista.wait(self._valor('calib_timeout_s'))
        finally:
            self._terminar_captura(captura)

        # Copia fija: un callback de profundidad que ya estaba en curso todavía puede agregar
        # un cuadro a la lista mientras se calcula.
        profundidades = list(captura.profundidades[:captura.cuadros])
        try:
            if captura.error:
                raise CalibrationError(captura.error)
            if not llego:
                raise CalibrationError(self._motivo_sin_datos(captura))
            resultado = calibrar_piso(
                profundidades, captura.intrinsecos, configuracion, vista,
                self._valor('warp_width'), self._valor('warp_height'))
        except CalibrationError as error:
            respuesta.message = 'calibración rechazada: %s' % error
            self.get_logger().warning(respuesta.message)
            return respuesta

        # Todos juntos: de a uno, una imagen podría procesarse con una pose a medio cambiar.
        # Pasan por la misma validación que cualquier cambio de parámetros.
        aplicado = self.set_parameters_atomically([
            Parameter(nombre, Parameter.Type.DOUBLE_ARRAY if isinstance(valor, list)
                      else Parameter.Type.DOUBLE, valor)
            for nombre, valor in resultado.parametros().items()])
        if not aplicado.successful:
            respuesta.message = 'calibración rechazada por el detector: %s' % aplicado.reason
            self.get_logger().warning(respuesta.message)
            return respuesta

        respuesta.success = True
        respuesta.message = '%s. Aplicada en vivo; para que quede al reiniciar, Guardar (%s)' % (
            resultado.resumen(), self._nombre(self._valor('save_service')))
        arriba = min(resultado.warp_src[1], resultado.warp_src[3])
        if arriba < self._valor('roi_top_fraction'):
            respuesta.message += (
                '. Aviso: el trapecio empieza en y=%.2f, más arriba que roi_top_fraction '
                '(%.2f): esa parte de la vista queda tapada. Bajá roi_top_fraction a %.2f o menos'
                % (arriba, self._valor('roi_top_fraction'), arriba))
        self.get_logger().info('calibración aplicada: %s' % respuesta.message)
        return respuesta

    def _al_guardar(self, _pedido, respuesta):
        """Guarda las medidas y los ajustes en `persist_file`, para que queden al reiniciar."""
        valores = {nombre: self._valor(nombre) for nombre in PARAMETROS_PERSISTENTES}
        try:
            ruta = guardar_parametros(self._valor('persist_file'), self.get_name(), valores)
        except OSError as error:
            respuesta.success = False
            respuesta.message = 'no se pudo guardar en %s: %s' % (
                self._valor('persist_file'), error)
            self.get_logger().error(respuesta.message)
            return respuesta
        respuesta.success = True
        respuesta.message = 'guardado en %s: %s' % (ruta, ', '.join(PARAMETROS_PERSISTENTES))
        self.get_logger().info(respuesta.message)
        return respuesta

    def _nombre(self, nombre):
        """El nombre completo de un servicio (expande el `~` del nodo)."""
        return self.get_fully_qualified_name() + nombre[1:] if nombre.startswith('~') else nombre

    def _empezar_chequeo(self):
        """Al arrancar: mide la cámara con unos pocos cuadros, sin bloquear el nodo."""
        self._chequeo = self._empezar_captura(self._valor('calib_check_frames'))
        self._inicio_chequeo = self.get_clock().now()
        self._timer_chequeo = self.create_timer(
            PERIODO_CHEQUEO_S, self._revisar_chequeo, callback_group=self._grupo_calibracion)

    def _revisar_chequeo(self):
        captura = self._chequeo
        reemplazada = self._captura is not captura
        vencido = (self.get_clock().now() - self._inicio_chequeo
                   > Duration(seconds=self._valor('calib_timeout_s')))
        if not (captura.lista.is_set() or vencido or reemplazada):
            return
        self._timer_chequeo.cancel()
        self._terminar_captura(captura)
        if reemplazada and not captura.lista.is_set():
            return   # una calibración tomó la cámara: ya no hace falta el chequeo
        logger = self.get_logger()
        if captura.error or not captura.lista.is_set():
            logger.info('no se pudo verificar la cámara al arrancar: %s'
                        % (captura.error or self._motivo_sin_datos(captura)))
            return
        try:
            pose = medir_pose(list(captura.profundidades[:captura.cuadros]),
                              captura.intrinsecos, self._config_calibracion())
        except CalibrationError as error:
            logger.warning('no se pudo verificar la cámara al arrancar: %s' % error)
            return
        altura, pitch = self._valor('calib_height_m'), self._valor('calib_pitch_deg')
        if (abs(pose.height_m - altura) > self._valor('calib_check_height_tol_m')
                or abs(pose.pitch_deg - pitch) > self._valor('calib_check_pitch_tol_deg')):
            logger.warning(
                'la cámara se movió desde la última calibración: altura %.3f m (calibrada '
                '%.3f m), pitch %.1f° (calibrado %.1f°). Recalibrá: ros2 service call %s '
                'std_srvs/srv/Trigger' % (pose.height_m, altura, pose.pitch_deg, pitch,
                                          self._nombre(self._valor('calibrate_service'))))
        else:
            logger.info('cámara en su lugar: altura %.3f m, pitch %.1f°'
                        % (pose.height_m, pose.pitch_deg))

    def _toca_depurar(self):
        """Indica si hay que dibujar las imágenes de depuración para esta imagen."""
        if not self._valor('publish_debug'):
            return False
        if (self._publicador_camara.get_subscription_count() == 0
                and self._publicador_cenital.get_subscription_count() == 0):
            return False
        ahora = self.get_clock().now()
        intervalo = Duration(seconds=1.0 / self._valor('debug_max_fps'))
        if self._ultima_depuracion is not None and ahora - self._ultima_depuracion < intervalo:
            return False
        self._ultima_depuracion = ahora
        return True

    def _on_image(self, msg: Image):
        try:
            bgr = imagen_a_bgr(msg.height, msg.width, msg.encoding, msg.step, msg.data)
        except ValueError as error:
            self.get_logger().warning('imagen descartada: %s' % error, throttle_duration_sec=5.0)
            return

        detector = self._detector
        depurar = self._toca_depurar()
        # El tiempo es el de la imagen: el fallback mide cuánto hace que no hay líneas
        # válidas en el reloj de la cámara, no en el del procesamiento.
        instante = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        previas = self._tracker.previas(instante)
        try:
            if depurar:
                analisis = detector.analizar(bgr, previas)
                resultado = analisis.resultado
            else:
                resultado = detector.detect(bgr, previas)
        except (cv2.error, np.linalg.LinAlgError) as error:
            self.get_logger().error(
                'imagen descartada: falló la visión: %s' % error, throttle_duration_sec=5.0)
            return

        carril = self._tracker.update(resultado, instante, self._velocidad_mps)

        estado = LaneState()
        estado.header = msg.header
        estado.mode = carril.mode
        estado.detected = carril.detected
        estado.left_valid = carril.left_valid
        estado.right_valid = carril.right_valid
        estado.lateral_error_m = carril.lateral_error_m
        estado.heading_error_rad = carril.heading_error_rad
        estado.curvature = carril.curvature
        # La fila de medición, en metros desde el pie de la cámara: el borde de abajo de la
        # vista más la fracción hacia adelante.
        estado.reference_distance_m = (
            self._valor('calib_forward_near_m')
            + self._valor('reference_row_fraction') * detector.config.view_length_m)
        estado.confidence = carril.confidence
        estado.intersection_detected = carril.intersection_detected
        self._publisher.publish(estado)

        if depurar:
            self._publicar_depuracion(detector, bgr, analisis, carril, msg.header)

    def _publicar_depuracion(self, detector, bgr, analisis, carril, encabezado):
        """Dibuja y publica las imágenes de depuración. Si falla, la detección sigue."""
        try:
            modo = NOMBRES_DE_MODO[carril.mode]
            filtrado = carril if self._valor('kalman_enabled') else None
            camara = dibujar_camara(detector, bgr, analisis, modo, filtrado)
            cenital = dibujar_cenital(detector, analisis, modo, filtrado)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warning(
                'no se pudo dibujar la depuración: %s' % error, throttle_duration_sec=5.0)
            return
        self._publicador_camara.publish(self._a_mensaje(camara, encabezado))
        self._publicador_cenital.publish(self._a_mensaje(cenital, encabezado))

    @staticmethod
    def _a_mensaje(imagen, encabezado):
        mensaje = Image()
        mensaje.header = encabezado
        mensaje.height, mensaje.width = imagen.shape[:2]
        mensaje.encoding = 'bgr8'
        mensaje.is_bigendian = 0
        mensaje.step = mensaje.width * 3
        mensaje.data = np.ascontiguousarray(imagen).tobytes()
        return mensaje


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    # Varios hilos: el servicio de calibración espera la profundidad sin frenar el resto.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
