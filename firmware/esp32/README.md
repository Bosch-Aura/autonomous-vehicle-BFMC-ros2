# firmware/esp32

Firmware del ESP32, responsable del control de bajo nivel del vehículo:

- Control de motores (PWM vía puente H) y de la dirección (servo)
- Lectura del sensor ultrasónico
- Comunicación serial con la Jetson, con el protocolo del [ADR-0001](../../docs/adr/0001-hardware-de-control-y-protocolo-serial.md)

## Estructura

```
esp32/
├── flash.sh                 # Compila y sube un sketch con arduino-cli
├── main/                    # Firmware principal
│   ├── main.ino             # Conecta pines y puerto serial con las reglas de protocol.h
│   └── protocol.h           # Protocolo y reglas puras, sin dependencias de Arduino
├── tests_host/              # Tests de protocol.h que corren en la computadora
│   ├── test_protocol.cpp
│   └── run_tests.sh
└── tests_componentes/       # Sketches de prueba por componente
    ├── test_motor/
    └── test_ultrasonico/
```

## Protocolo con la Jetson

Un objeto JSON por línea, terminada en `
`, a 115200 baudios. El detalle y el motivo están en el ADR-0001.

| Sentido | Mensaje | Campos |
|---|---|---|
| Jetson → ESP32 | `drive` | `seq`, `v` (mm/s, + avanza), `s` (décimas de grado, + izquierda) |
| Jetson → ESP32 | `brake` | `seq` |
| ESP32 → Jetson | `ultra` (20 Hz) | `id`, `dist_cm`, `valid` |
| ESP32 → Jetson | `status` (2 Hz y ante cada cambio) | `fw`, `wd`, `last_seq` |

Una línea que no se puede interpretar (truncada, sin campos, con tipos incorrectos o demasiado larga) se **descarta sin cortar el enlace** y sin reiniciar el watchdog.

## Seguridad: watchdog

Si pasan 300 ms sin un mensaje válido, el firmware corta los motores y centra la dirección, aunque la Jetson haya dejado de funcionar. Retoma al recibir el siguiente mensaje válido. Mientras está cortado, `status` informa `"wd":true`. Antes del primer mensaje el auto ya está detenido.

## Pines

| Función | Pin | Origen |
|---|---|---|
| Puente H: IN1, IN2, ENA | GPIO 14, 12, 13 | `tests_componentes/test_motor` |
| Ultrasónico: TRIG, ECHO | GPIO 26, 27 | `tests_componentes/test_ultrasonico` |
| Servo de dirección | GPIO 25 | **Asumido**: ningún sketch de prueba lo define |

## Valores asumidos, a confirmar en el auto

Están marcados con `[asumido]` en `main.ino`:

- GPIO 25 para el servo de dirección.
- 115200 baudios y watchdog de 300 ms (ADR-0001).
- Rango de ±500 mm/s y ±25,0° (ADR-0001).
- Servo con centro en 1500 µs y ±400 µs de amplitud. Si el servo está invertido, la amplitud va negativa.

## Tests en la computadora

Las reglas de `protocol.h` (parser, formato de los mensajes, watchdog y conversión de unidades) se testean sin ESP32:

```bash
./tests_host/run_tests.sh
```

Necesita `g++`. Dentro del contenedor: `docker compose run --rm dev bash firmware/esp32/tests_host/run_tests.sh`. El CI los corre en cada PR.

Lo que depende de los cables (pines, PWM, servo, ultrasónico) no se puede testear así: se prueba en el auto, primero con las ruedas en el aire.

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
- `g++`, solo para los tests de la computadora
