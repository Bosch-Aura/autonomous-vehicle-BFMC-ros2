# firmware/esp32

Firmware del ESP32, responsable del control de bajo nivel del vehículo:

- Control de motores (PWM vía puente H)
- Lectura del sensor ultrasónico
- Comunicación serial con la Jetson

## Estructura

```
esp32/
├── flash.sh                 # Compila y sube un sketch con arduino-cli
├── main/                    # Firmware principal
│   └── main.ino
└── tests_componentes/       # Sketches de prueba por componente
    ├── test_motor/
    └── test_ultrasonico/
```

## Cómo flashear

Conectar el ESP32 por USB y correr, desde esta carpeta:

```bash
./flash.sh                          # sube main/main.ino
./flash.sh test_motor               # sube un sketch de tests_componentes/
./flash.sh test_motor /dev/ttyUSB1  # elige el puerto a mano
```

El script:

1. Revisa que `arduino-cli` esté instalado (si no, muestra cómo instalarlo).
2. Instala el soporte para ESP32 la primera vez.
3. Busca el puerto del ESP32 (si hay más de uno, pregunta cuál).
4. Detecta el modelo de ESP32 (si no puede, lo pregunta). Para forzarlo: `FQBN=esp32:esp32:esp32s3 ./flash.sh`.
5. Compila, sube y abre el monitor serial a 115200 baudios (`Ctrl+C` para salir).

Si dice que no hay permiso sobre el puerto: `sudo usermod -aG dialout $USER` y volver a iniciar sesión.

También se puede subir desde el Arduino IDE, abriendo el `.ino` y eligiendo la placa y el puerto.

## Dependencias

- [arduino-cli](https://arduino.github.io/arduino-cli/latest/installation/) (o el Arduino IDE)
