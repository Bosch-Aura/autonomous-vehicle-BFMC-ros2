# Estado del carril estimado por bfmc_lane_detection.
# Unidades SI. Marco del vehículo: x hacia adelante, y hacia la izquierda.

std_msgs/Header header

# false si no se detectó el carril en este cuadro. Los demás campos no son válidos.
bool detected

# Desvío lateral [m]. Positivo: el centro del carril queda a la izquierda del auto
# (hay que girar a la izquierda).
float32 lateral_error_m

# Rumbo del carril respecto del auto [rad]. Positivo: el carril sigue hacia la izquierda.
float32 heading_error_rad

# Confianza de la detección, de 0.0 a 1.0.
float32 confidence

# true si se detectó una línea transversal (intersección) por delante.
bool intersection_detected
