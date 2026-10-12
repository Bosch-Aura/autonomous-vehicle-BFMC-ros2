# Guía de tipos de test

Apoyo para el paso 1 de la skill `escribir-tests`. Cada tipo responde a una pregunta distinta, cuesta distinto y falla por motivos distintos.

## Unitario

- **Pregunta:** ¿esta regla de la lógica da el resultado correcto con datos conocidos?
- **Qué cubre en este proyecto:** PID y filtros, conversiones de unidades, máquinas de estado de `bfmc_decision`, secuencias de las maniobras, cálculo de estado del carril.
- **Cómo es:** sin ROS, sin hardware y con tiempo simulado. Corre en milisegundos.
- **Cuándo falla:** cuando cambia una regla. Si falla por un cambio interno que no altera el comportamiento, el test está atado a la implementación y conviene reescribirlo.
- **Ejemplo:** [`ejemplos/test_unitario_tiempo_simulado.py`](ejemplos/test_unitario_tiempo_simulado.py).

## Contrato

- **Pregunta:** ¿lo que este módulo le manda al otro respeta el formato acordado?
- **Qué cubre:** los comandos hacia el ESP32 (enteros, mm/s y décimas de grado, dentro de rango), los campos de los mensajes propios, los nombres de topics y los tipos que cada nodo publica y consume según `docs/ARQUITECTURA.md`.
- **Cómo es:** unitario, pero la referencia es el acuerdo entre módulos y no la lógica interna. Conviene barrer entradas extremas, no solo los casos felices.
- **Cuándo falla:** cuando alguien cambia un formato sin avisar al otro lado. Es el test que más protege la integración.
- **Ejemplo:** [`ejemplos/test_contrato_comandos.py`](ejemplos/test_contrato_comandos.py).

## Integración de nodo

- **Pregunta:** ¿el nodo conecta bien sus mensajes con su lógica y lee sus parámetros?
- **Cómo es:** crea el nodo real y un nodo auxiliar, publica un mensaje y espera la salida con un tope de tiempo. Usa los parámetros reales de `config/params.yaml`, así el test falla si el YAML y el código se desincronizan.
- **Costo:** más lento que un unitario y depende de la mensajería. Por eso se usa para verificar la conexión, no las reglas de negocio.
- **Ejemplo:** [`ejemplos/test_nodo_ros.py`](ejemplos/test_nodo_ros.py).

## Integración con rosbag

- **Pregunta:** ¿varias piezas juntas se comportan bien sobre datos reales de la pista?
- **Cómo es:** se reproduce una grabación (`ros2 bag play`) y se verifica lo que publican los nodos, por ejemplo que `lane_detection` produzca un estado de carril válido en todos los cuadros.
- **Estado:** todavía no hay una grabación en el repositorio, así que esta guía no trae un ejemplo ejecutable. Cuando exista, un test de este tipo debería declarar la grabación que usa y qué comportamiento verifica. El patrón de arriba es una sugerencia sin verificar.

## Estilo

Cada paquete generado con `nuevo-nodo` trae `test_flake8.py`, que aplica PEP 8. No hace falta escribirlo a mano.

## Qué no vale la pena testear

- Getters y setters triviales.
- El funcionamiento interno de ROS2 o de pytest.
- Detalles de implementación que cambian con cada refactor (nombres de variables privadas, orden de llamadas internas).
- Valores que dependen del auto real y no se pueden reproducir en un contenedor: se documentan como pendientes de prueba en el hardware.

## Nombres de tests

El nombre describe una regla del sistema, no una función. Comparar:

- `test_watchdog_1`: no dice nada.
- `test_al_cumplirse_el_tiempo_limite_se_vence`: documenta el comportamiento. Si falla, el nombre ya explica qué dejó de cumplirse.

## Tiempo en punto flotante

Al simular el tiempo conviene usar valores que se representen exactos, como potencias de 2 (`0.5`, `0.125`). Una resta como `100.3 - 100.0` da `0.29999999999999716`, y `(100.3 - 100.0) >= 0.3` es falso. Un test así falla según el instante elegido, sin que la lógica tenga ningún error.
