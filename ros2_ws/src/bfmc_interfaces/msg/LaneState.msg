# Estado del carril estimado por bfmc_lane_detection.
# Unidades SI. Marco del vehículo: x hacia adelante, y hacia la izquierda.
# El header es el de la imagen de la que sale la medición.

# Modo: de dónde sale el estado del carril (fallback escalonado).
uint8 MODE_TWO_LINES=0    # las dos líneas pasaron los chequeos: centro = promedio
uint8 MODE_ONE_LINE=1     # una sola línea válida: centro = esa línea ± medio ancho de carril
uint8 MODE_PREDICTED=2    # ninguna válida hace poco: se repite el último estado válido
uint8 MODE_LOST=3         # ninguna válida por más de lost_timeout_s: no se sabe dónde está

std_msgs/Header header

uint8 mode

# false solo en MODE_LOST. En ese caso los demás campos no son válidos (NaN y confianza 0).
bool detected

# Si cada línea pasó sus chequeos en este cuadro.
bool left_valid
bool right_valid

# Desvío lateral [m]. Positivo: el centro del carril queda a la izquierda del auto
# (hay que girar a la izquierda).
float32 lateral_error_m

# Rumbo del carril respecto del auto [rad]. Positivo: el carril sigue hacia la izquierda.
float32 heading_error_rad

# Confianza de la detección, de 0.0 a 1.0.
float32 confidence

# true si se detectó una línea transversal (intersección) por delante.
bool intersection_detected
