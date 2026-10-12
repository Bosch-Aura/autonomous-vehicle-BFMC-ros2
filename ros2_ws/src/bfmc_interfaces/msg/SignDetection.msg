# Una señal detectada por bfmc_sign_detection.

# Tipo de señal, por ejemplo "stop" o "priority". La correspondencia con las clases
# del modelo está en el params.yaml de bfmc_sign_detection.
string label

# Confianza de la detección, de 0.0 a 1.0.
float32 confidence

# Distancia a la señal [m]. NaN si no se pudo estimar.
float32 distance_m

# Rectángulo que encierra la señal, en píxeles de la imagen de color.
uint16 x_min
uint16 y_min
uint16 x_max
uint16 y_max
