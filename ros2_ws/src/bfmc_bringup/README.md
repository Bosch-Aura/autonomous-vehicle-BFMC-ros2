# bfmc_bringup

Arranca todo el sistema con un solo comando. No tiene nodos propios: lanza los de los demás paquetes, cada uno con el `config/params.yaml` de **su** paquete. Acá no hay parámetros duplicados.

## Cómo usarlo

```bash
ros2 launch bfmc_bringup system.launch.py
```

| Opción | Por defecto | Qué hace |
|---|---|---|
| `camera` | `true` | Arranca el driver de la cámara RealSense (`bfmc_camera`) |
| `signs` | `false` | Arranca la detección de señales. Necesita Ultralytics y el modelo: ver el README de `bfmc_sign_detection` |
| `serial_port` | el de `bfmc_hardware_bridge` | Puerto serial del ESP32 |

```bash
ros2 launch bfmc_bringup system.launch.py camera:=false             # sin cámara (por ejemplo, para probar)
ros2 launch bfmc_bringup system.launch.py signs:=true               # con detección de señales
ros2 launch bfmc_bringup system.launch.py serial_port:=/dev/ttyACM0 # otro puerto
```

## Qué arranca

| Paquete | Nodo | Opcional |
|---|---|---|
| `bfmc_camera` | driver `realsense2_camera` | `camera` |
| `bfmc_lane_detection` | `lane_detection` | |
| `bfmc_sign_detection` | `sign_detection` | `signs` |
| `bfmc_control` | `control` | |
| `bfmc_hardware_bridge` | `hardware_bridge` | |
| `bfmc_maneuvers` | `maneuvers` | |
| `bfmc_decision` | `decision` | |

**Sin ninguna entrada, el sistema queda detenido:** `bfmc_decision` en `STARTING` y el comando final en cero. Empieza a conducir recién cuando llegan las primeras entradas del hardware y del comando de carril.

La detección de señales arranca **desactivada** porque necesita Ultralytics y el modelo descargado.

## Qué verifican los tests

Son pruebas de **integración**: comprueban lo que ningún nodo puede verificar por sí solo.

- **El launch:** qué nodos arranca, cuáles son opcionales, los valores por defecto de las opciones y que cada paquete lanzado figure como dependencia.
- **Coherencia entre los YAML** (`test_contracts.py`). Cada paquete declara los nombres de los topics que usa en su propio YAML y ninguno conoce a los demás: si dos que deben hablarse tienen nombres distintos, el sistema arranca pero **no se comunica, sin ningún error**. Los tests comparan los nombres de topics y de la acción, las etiquetas de señal que usa `bfmc_decision` contra las que produce el detector, y que los límites del puente (velocidad y giro) cubran lo que piden el control y las maniobras.
- **Arranque real** (`test_startup.py`): lanza el sistema completo con los YAML reales, sin cámara ni ESP32, y comprueba que arranquen todos los nodos, que ninguno muera, que `bfmc_decision` quede en el estado de arranque y que el comando final sea cero. Corre en un dominio de ROS propio para no cruzarse con los tests de otros paquetes.

```bash
docker compose run --rm test
```

**Lo que no verifican:** que el auto se mueva bien. Eso necesita el hardware.

## Si cambiás un topic

Cambialo en el YAML de **todos** los paquetes que lo mencionan: `test_contracts.py` falla si quedan distintos y dice cuáles.
