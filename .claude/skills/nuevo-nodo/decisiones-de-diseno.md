# Decisiones de diseño: pros y contras

Catálogo para el paso 2 de la skill `nuevo-nodo`. Cada decisión se presenta como propuesta: la persona elige. Las recomendaciones valen para este proyecto y pueden cambiar si el caso lo justifica.

## Separar la lógica del nodo

- **Qué se propone:** la lógica va en una clase sin `rclpy`; el nodo solo conecta mensajes con ella.
- **Pros:** se testea sin ROS, rápido y con tiempo simulado; se puede reutilizar y reemplazar sin tocar los mensajes.
- **Contras:** un archivo más y algo de código de pegamento. En un nodo de tres líneas puede ser excesivo.
- **Se recomienda:** siempre que haya reglas de negocio (umbrales, estados, cálculos). Se puede omitir si el nodo solo reenvía datos.

## Nombres de topics como parámetros o con remapping

- **Qué se propone:** el nombre del topic es un parámetro en `config/params.yaml`.
- **Pros:** todo valor configurable vive en un solo lugar y se ve en el README; coincide con la regla "nada hardcodeado".
- **Contras:** el remapping (`--ros-args -r`) es la forma idiomática de ROS2 y las herramientas lo entienden mejor.
- **Se recomienda:** parámetros, por coherencia con el resto del proyecto. Es una convención del repositorio, no un requisito de ROS2.

## QoS de los topics

- **Datos de sensores continuos (imágenes, ultrasónico):** *best effort*, profundidad chica. Pro: no se acumula retraso si un consumidor se atrasa. Contra: puede perder mensajes.
- **Comandos y estados (`/cmd/drive`, `/vehicle/state`):** *reliable*, profundidad chica. Pro: no se pierde una orden. Contra: ante un enlace lento puede encolar y entregar datos viejos.
- **Se recomienda:** elegir explícitamente en cada publicador y suscriptor. Un publicador *best effort* y un suscriptor *reliable* no se conectan, y el error es silencioso.

## Procesar en el callback o con un timer

- **En el callback:** la salida llega apenas llega la entrada. Pro: simple y con mínima latencia. Contra: si la entrada llega más rápido de lo que se procesa, se acumula trabajo.
- **Con un timer a frecuencia fija:** el nodo se ejecuta a un ritmo predecible. Pro: facilita razonar sobre tiempos y watchdogs. Contra: agrega latencia de hasta un período y exige guardar el último dato.
- **Se recomienda:** callback para filtros y transformaciones; timer para control, decisión y todo lo que deba reaccionar aunque no lleguen mensajes.

## Un nodo o dos

- **Un nodo:** menos mensajes y menos configuración. Contra: mezcla responsabilidades y cuesta testear cada una.
- **Dos nodos:** cada uno cambia por una sola razón. Contra: más topics, más latencia y más archivos de lanzamiento.
- **Se recomienda:** dos si la descripción necesita una "y" o si una parte tiene un ritmo o una criticidad distinta. Uno si no.

## Tipos de mensaje estándar o propios

- **Estándar** (`sensor_msgs`, `ackermann_msgs`): herramientas como `rviz` y `ros2 bag` ya los entienden; no hay paquete extra.
- **Propios** (`bfmc_interfaces`): permiten campos específicos (confianza, distancia, enums de señal). Contra: obligan a compilar un paquete más y a versionarlo.
- **Se recomienda:** estándar siempre que alcance; propio solo cuando falten campos.

## Maniobras: patrón Strategy o `if/elif`

- **Strategy** (una clase por maniobra): agregar una maniobra no toca las otras (principio abierto/cerrado) y cada una se testea sola. Contra: más archivos y una interfaz que mantener.
- **`if/elif` en un solo lugar:** menos estructura al principio. Contra: crece sin control, cada cambio puede romper otra maniobra y el archivo se vuelve difícil de testear.
- **Se recomienda:** Strategy desde la segunda maniobra. Para una sola, es un costo de arranque razonable porque se espera que haya más.

## Parámetros fijos en YAML o modificables en vivo

- **YAML cargado al arrancar:** simple y reproducible; el comportamiento queda registrado en el repositorio. Contra: cambiar un valor exige relanzar.
- **Parámetros dinámicos (`ros2 param set`):** permiten sintonizar sin reiniciar, útil para PID. Contra: hay que validar el valor en cada cambio y el ajuste se pierde si no se vuelca al YAML.
- **Se recomienda:** YAML como fuente de verdad. Habilitar cambios en vivo solo donde la sintonía lo justifique, y volcar el valor final al YAML.
