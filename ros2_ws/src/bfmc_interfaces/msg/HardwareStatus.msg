# Estado del ESP32 y del enlace con él, publicado por bfmc_hardware_bridge.

std_msgs/Header header

# true si llegaron mensajes del ESP32 dentro del tiempo límite.
bool link_up

# true si el ESP32 está en FAULT: lo cortó el watchdog (falta de comandos) o el E-STOP.
bool watchdog_tripped

# Versión del firmware que reporta el ESP32. Vacío si todavía no reportó.
string firmware_version

# Número de secuencia del último comando que el ESP32 confirmó haber recibido.
uint32 last_seq

# true si el ESP32 bloquea el avance porque el ultrasónico vio un obstáculo cerca. Lo decide el
# firmware, no la Jetson: frena solo y se libera con lecturas lejanas válidas o con
# M:OBSTACLE_RESET (ADR-0003).
bool obstacle_stop

# Distancia a la que el ESP32 detectó el obstáculo [m]. NaN si no hay obstáculo o si el
# firmware no informó la distancia.
float32 obstacle_distance_m

# true si el ultrasónico lleva 2 s sin ninguna lectura válida: espacio abierto o sensor
# desconectado. Es informativo; con el auto frenado por un obstáculo, el puente lo usa para
# destrabarlo solo.
bool ultrasonic_silent

# Estado que reporta el ESP32: DISARMED, RUNNING, STOPPED o FAULT. Vacío si todavía no
# informó ninguno. STOPPED es un auto armado pero quieto, por un pedido de detener o por un
# obstáculo (en ese caso obstacle_stop también es true).
string esp32_state
