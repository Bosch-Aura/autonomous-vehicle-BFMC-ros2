# firmware/esp32

Firmware del ESP32, responsable del control de bajo nivel del vehículo:

- Control de motores (PWM vía puente H)
- Lectura del sensor ultrasónico
- Comunicación serial con la Jetson

## Estructura

```
esp32/
├── main/                    # Firmware principal
│   └── main.ino
└── tests_componentes/       # Sketches de prueba por componente
    ├── test_motor/
    └── test_ultrasonico/
```

## Cómo flashear

Usar el script `flash.sh` en la raíz del repositorio, o cargar directamente desde el Arduino IDE.

## Dependencias

- Arduino IDE o PlatformIO
- Librería `NewPing` (sensor ultrasónico)
