# Estado de seguridad publicado por bfmc_safety. Informa: no frena por sí mismo.

uint8 REASON_NONE=0
uint8 REASON_OBSTACLE=1
uint8 REASON_SENSOR_INVALID=2
uint8 REASON_SENSOR_STALE=3

std_msgs/Header header

# true si el auto debe detenerse.
bool emergency

# Motivo de la emergencia (REASON_*). REASON_NONE si no hay emergencia.
uint8 reason

# true si la última lectura del sensor es válida y reciente.
bool sensor_valid

# Distancia al obstáculo [m]. NaN si no hay lectura válida.
float32 distance_m
