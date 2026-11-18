# Simulador de carril (lazo cerrado)

Prueba la **lógica** del carril sin el auto: dibuja la vista cenital que vería el auto en una pista
sintética, la pasa por el detector, el tracker y el control **reales** de `ros2_ws/src` (con sus
`params.yaml`) y mueve el auto con un modelo de bicicleta. Análisis y resultados:
[`docs/ANALISIS-CARRIL-Y-CONTROL.md`](../../docs/ANALISIS-CARRIL-Y-CONTROL.md).

No modela la cámara real (luz, sombras, perspectiva) ni el servo: sirve para encontrar fallas de
lógica, no para afinar números.

## Cómo correrlo

No necesita ROS. Necesita Python 3 con numpy, opencv y pyyaml:

```bash
python3 -m venv ~/.venvs/sim && ~/.venvs/sim/bin/pip install numpy opencv-python-headless pyyaml
cd herramientas/simulador_carril
~/.venvs/sim/bin/python casos.py             # con los params.yaml del repo
~/.venvs/sim/bin/python casos.py --antes     # como antes del 2026-11-17
~/.venvs/sim/bin/python casos.py --filtro "C der"
```

Cada línea dice si el auto se sale (`SALE`), el peor desvío, el % de cuadros en cada modo del
carril (dos líneas, una, predicho, perdido), cuánto tiempo seguido estuvo perdido (la decisión
frena), si se detectó intersección y cuántos cuadros midió el centro del lado equivocado.

## Supuestos (cambiarlos en `sim.py` / `Marcas` si se miden)

Línea de 2 cm, punteada de 10 cm cada 10 cm, latencia 0,1 s, 30 cuadros por segundo, y las medidas
del auto de los `params.yaml` (`wheelbase_m`, `camera_to_rear_axle_m`).
