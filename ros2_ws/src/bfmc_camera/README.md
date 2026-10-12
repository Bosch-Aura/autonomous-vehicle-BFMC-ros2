# bfmc_camera

Configura el driver oficial de la RealSense (`realsense2_camera`) con los nombres de topic del sistema. No tiene código propio: escribir otro driver sería reinventar uno que ya existe y se mantiene.

## Responsabilidad

Entregar imágenes de color y de profundidad con nombres de topic fijos. No procesa las imágenes.

## Topics

| Dirección | Topic | Tipo | Descripción |
|---|---|---|---|
| Publica | `/camera/color/image_raw` | `sensor_msgs/Image` | Imagen de color |
| Publica | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Profundidad **alineada al color** |

Los demás topics del driver (infrarrojo, información de cámara, la profundidad sin alinear en `/camera/depth/image_rect_raw`, etc.) también se publican, pero el sistema no los usa.

**Por qué la profundidad alineada:** `bfmc_sign_detection` mide la distancia a una señal en el píxel de la imagen de color donde la detectó. Con la profundidad sin alinear, ese píxel no corresponde al mismo punto de la escena.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml) y son los del driver. Los que importan:

| Parámetro | Valor | Descripción |
|---|---|---|
| `rgb_camera.color_profile` | `640x480x30` | Ancho x alto x cuadros por segundo del color. **Asumido**: depende del modelo de cámara. |
| `depth_module.depth_profile` | `640x480x30` | Igual para la profundidad. **Asumido**. |
| `align_depth.enable` | `true` | Publica la profundidad alineada al color. |
| `enable_sync` | `true` | Sincroniza los cuadros de color y profundidad. |
| `initial_reset` | `false` | Reinicia la cámara al arrancar. Poner `true` si se cuelga. |

## Cómo usarlo

Con la cámara conectada a un puerto **USB 3**:

```bash
ros2 launch bfmc_camera camera.launch.py
```

Para comprobar que llegan las imágenes:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

Si no aparece ninguna cámara: `rs-enumerate-devices` debe listarla. Revisá el cable y que el puerto sea USB 3.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests no necesitan la cámara. Verifican que el launch arranque el driver oficial, que el YAML esté bien formado, que la profundidad se alinee al color y que **cada parámetro exista en el driver**: un nombre mal escrito se ignoraría en silencio. Que la cámara entregue imágenes con los nombres esperados solo se puede comprobar con la cámara conectada.
