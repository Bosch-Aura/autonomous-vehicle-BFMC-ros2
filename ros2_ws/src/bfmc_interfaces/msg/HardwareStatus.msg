# Estado del enlace con el ESP32, publicado por bfmc_hardware_bridge.

std_msgs/Header header

# true si llegaron mensajes del ESP32 dentro del tiempo límite.
bool link_up

# true si el watchdog del ESP32 cortó los motores por falta de comandos.
bool watchdog_tripped

# Versión del firmware que reporta el ESP32. Vacío si todavía no reportó.
string firmware_version

# Número de secuencia del último comando que el ESP32 confirmó haber recibido.
uint32 last_seq
