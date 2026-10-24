# Estado del ESP32 y del enlace con él, publicado por bfmc_hardware_bridge.

std_msgs/Header header

# true si llegaron mensajes del ESP32 dentro del tiempo límite.
bool link_up

# true si el watchdog del ESP32 cortó los motores por falta de comandos.
bool watchdog_tripped

# Versión del firmware que reporta el ESP32. Vacío si todavía no reportó.
string firmware_version

# Número de secuencia del último comando que el ESP32 confirmó haber recibido.
uint32 last_seq

# true si el ESP32 está detenido porque el ultrasónico vio un obstáculo cerca (STOPPED por
# OBSTACLE). Lo decide el firmware, no la Jetson: frena solo y se libera solo cuando se despeja.
bool obstacle_stop

# Distancia a la que el ESP32 detectó el obstáculo [m]. NaN si no hay obstáculo o si el
# firmware no informó la distancia.
float32 obstacle_distance_m
