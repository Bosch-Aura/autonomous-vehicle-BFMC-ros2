# Estado del vehículo publicado por bfmc_decision, para el dashboard y el diagnóstico.

uint8 STATE_STARTING=0
uint8 STATE_LANE_FOLLOW=1
uint8 STATE_MANEUVER=2
uint8 STATE_EMERGENCY=3

std_msgs/Header header

# Estado actual de la máquina de estados (STATE_*).
uint8 state

# Nombre de la maniobra en curso. Vacío si no hay ninguna.
string active_maneuver

# Texto breve que explica el estado actual, para humanos.
string reason
