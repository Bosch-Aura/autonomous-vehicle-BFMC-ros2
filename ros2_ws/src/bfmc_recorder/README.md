# bfmc_recorder

Graba lo que ve y hace el auto para repetirlo fuera de él: probar otros parámetros del detector de carril y del control con las mismas imágenes. **No maneja el auto**: solo escucha.

## Cómo se usa

1. Lanzar el sistema (`ros2 launch bfmc_bringup system.launch.py`): el nodo `recorder` arranca sin grabar.
2. En el visor, tarjeta **Grabación**: **Grabar** y, al terminar, **Parar grabación**. Para solo, a los 10 minutos o con menos de 500 MB libres.
3. Al parar, el visor muestra dos comandos para bajar la carpeta a la PC y borrarla de la Jetson.

Sin visor: `ros2 service call /recorder/start std_srvs/srv/Trigger` y `/recorder/stop`.

## Qué queda en `~/bfmc_grabaciones/<fecha_hora>/`

| Archivo | Contenido |
|---|---|
| `frames/<instante_ns>.jpg` | cuadros de la cámara sin procesar (`/camera/color/image_raw`), con la hora de la imagen |
| `events.jsonl` | una línea por mensaje de `lane` (`/perception/lane`), `lane_cmd` (`/control/lane_cmd`), `drive` (`/cmd/drive`) y `hardware` (`/hardware/status`), con `recv_ns` y `stamp_ns` |
| `meta.json` | inicio, fin, motivo del fin, ritmo, calidad, cuadros, eventos y bytes |
| `params_lane_detection.yaml`, `params_control.yaml` | `ros2 param dump` de los nodos al empezar: con qué parámetros se grabó |
| `meta_lane_detection.yaml` | el que guardó el visor con **Guardar**, si existe |

Son archivos comunes (JPEG y JSON): se leen sin ROS. A 10 cuadros por segundo son unos 30–40 MB por minuto.

## Repetir una grabación por el detector (`repetir_carril.py`)

Pasa los cuadros por el mismo detector y tracker que el nodo `lane_detection`, con otros parámetros, sin ROS corriendo ni el auto. Hace falta `bfmc_lane_detection` instalado o en `PYTHONPATH`.

```
python3 -m bfmc_recorder.repetir_carril GRABACION detector=l_adaptativo l_offset=20
python3 -m bfmc_recorder.repetir_carril GRABACION --grilla calib_half_width_m=0.30,0.36 lane_width_tolerance_m=0.05,0.08 detector=l_adaptativo
```

Cada `nombre=valor` pisa un parámetro (los demás salen de `params_lane_detection.yaml` de la grabación). Con `--grilla`, los que traen varios valores separados por coma se combinan todos con todos. Imprime por combinación el porcentaje de cuadros con dos líneas, una, predicho y perdido, los cambios de lado, los saltos de rumbo mayores a 0,4 rad y el desvío medio.

## Parámetros (`config/params.yaml`)

| Parámetro | Unidad | Qué hace |
|---|---|---|
| `image_topic`, `lane_topic`, `lane_cmd_topic`, `drive_topic`, `hardware_status_topic` | | Qué se graba |
| `output_dir` | | Carpeta en la Jetson |
| `max_fps` | Hz | Cuadros por segundo que se guardan (0 = todos) |
| `jpeg_quality` | 1–100 | Calidad del JPEG |
| `max_minutes` | min | Corta sola |
| `min_free_mb` | MB | Corta, o no empieza, con menos espacio libre |
| `start_service`, `stop_service`, `status_topic` | | Servicios `std_srvs/Trigger` y estado (JSON en `std_msgs/String`, 1 Hz) que usa el visor |
| `meta_files`, `dump_nodes` | | Qué se copia y de qué nodos se vuelcan los parámetros |
