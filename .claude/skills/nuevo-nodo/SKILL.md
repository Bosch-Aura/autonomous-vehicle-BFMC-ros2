---
name: nuevo-nodo
description: Usar cuando haya que agregar un nodo, un paquete ROS2 o una maniobra al workspace. Propone el diseño (responsabilidad, topics, parámetros), pide confirmación y arma el esqueleto con su test y su documentación respetando la arquitectura del proyecto.
---

# Nuevo nodo o maniobra

Guía a quien agrega una funcionalidad al vehículo y arma el esqueleto del paquete. El objetivo es doble: que el código respete la arquitectura para que el repositorio no se vuelva a acoplar, y que quien lo escribe entienda por qué cada decisión es como es.

## Reglas que valen en toda la skill

- **Proponer, no imponer.** El diseño es una propuesta. La persona decide sí o no y puede refutarla.
- **No ceder por complacencia.** Si hay un desacuerdo, verificar contra la documentación o el código antes de responder. Nunca cambiar de postura solo porque se insiste. Sostener lo correcto con evidencia, y corregir lo propio solo si la evidencia lo muestra.
- **Objetividad con datos.** Contrastar contra fuentes reales (`docs/ARQUITECTURA.md`, el código del repositorio, la documentación de ROS2), no contra suposiciones. Lo que no se pudo comprobar se dice como tal.
- **Explicar el porqué.** Cada decisión de diseño se presenta con pros y contras. El catálogo está en [`decisiones-de-diseno.md`](decisiones-de-diseno.md).

## Antes de proponer nada

1. Leer `docs/ARQUITECTURA.md` (principios, paquetes, contratos, estructura de un paquete) y `CLAUDE.md`.
2. Listar `ros2_ws/src/` y leer el `README.md` de los paquetes cercanos, para no duplicar una responsabilidad que ya existe.
3. Si lo pedido contradice la arquitectura, decirlo citando la sección y proponer una alternativa. No generar nada hasta resolverlo.

## Paso 1: clasificar

Preguntar si se trata de:

- **Un nodo o paquete nuevo** (percepción, control, seguridad, puente de hardware...): sigue el flujo de este documento.
- **Una maniobra** (stop, intersección...): es una estrategia dentro de `bfmc_maneuvers`, no un paquete nuevo. Ver la sección "Maniobras".

## Paso 2: proponer el diseño (obligatorio, antes de generar)

Presentar la propuesta completa y esperar un sí explícito. No se genera ningún archivo antes.

1. **Responsabilidad en una frase.** Si hace falta una "y" para describirla, proponer dos nodos y explicar por qué.
2. **Topics.** Tabla con dirección, nombre, tipo de mensaje y QoS. Indicar si ya figura en `docs/ARQUITECTURA.md` §5; si es nuevo, proponer agregarlo ahí en el mismo cambio. Los nodos se comunican solo por topics, sin importar código de otros paquetes.
3. **Parámetros.** Nombre, tipo, unidad, valor propuesto y por qué ese valor. Nada de valores mágicos en el código.
4. **Separación.** Qué va en la lógica pura (sin ROS) y qué en el nodo. Qué entra como `now` para poder simular el tiempo.
5. **Tests.** Una línea por regla que se va a verificar, con el caso que cubre y por qué aporta.
6. **Decisiones con pros y contras.** Como mínimo dos de las del catálogo que realmente apliquen al caso.

Cerrar con: "¿Lo genero así? Podés cambiar cualquier punto."

## Paso 3: generar el esqueleto

Desde la raíz del repositorio:

```bash
python .claude/skills/nuevo-nodo/render.py nodo \
    --paquete bfmc_<funcionalidad> --nombre <nombre> --clase <Clase> \
    --descripcion "<una frase, sin comillas>"
```

Crea `ros2_ws/src/<paquete>/` con `package.xml`, `setup.py`, lógica pura, nodo, `config/params.yaml`, launch, tests (lógica, nodo y estilo) y `README.md`. Nunca sobrescribe lo que ya existe.

El esqueleto trae un ejemplo mínimo que **compila y pasa sus tests tal cual**: limita un valor a un rango. Sirve para mostrar el patrón. Hay que reemplazarlo por lo acordado en el paso 2, manteniendo la forma:

- Lógica en `<nombre>_logic.py`, sin importar `rclpy`, con el tiempo como argumento.
- Nodo en `<nombre>_node.py`: declara parámetros **sin valor por defecto**, crea subscripciones y publicaciones y llama a la lógica.
- Valores en `config/params.yaml`.
- Dependencias nuevas (tipos de mensaje) en `package.xml`.
- Los tests se reescriben con las reglas reales: el nombre de cada test describe una regla del sistema.

## Paso 4: documentar

- Completar el `README.md` del paquete: responsabilidad, topics, parámetros y cómo probarlo.
- Si hay topics nuevos o cambió un contrato, actualizar las tablas de `docs/ARQUITECTURA.md` §4 y §5 en el mismo cambio.

## Paso 5: verificar

```bash
docker compose run --rm test
```

Mostrar el resultado. Si Docker no está disponible, decir explícitamente que no se verificó. Antes de dar el trabajo por terminado:

- [ ] Compila (`colcon build`) y pasan todos los tests, incluido el de estilo.
- [ ] No hay imports de código de otros paquetes.
- [ ] No hay valores hardcodeados: todo parámetro está en `config/params.yaml`.
- [ ] La lógica no llama a `time.sleep` ni lee el reloj.
- [ ] El paquete tiene `README.md` y los contratos de `docs/ARQUITECTURA.md` están al día.
- [ ] Lo que depende del auto real (serial, cámara, latencias) figura como no verificado.

## Maniobras

Una maniobra es una clase que implementa `ManeuverStrategy` (patrón Strategy) dentro de `bfmc_maneuvers`. Cada maniobra tiene una sola razón para cambiar y agregar una nueva no obliga a tocar las anteriores.

1. Verificar que `ros2_ws/src/bfmc_maneuvers/` exista. Si no, crearlo primero con el flujo de nodo (`--paquete bfmc_maneuvers`).
2. Proponer el diseño como en el paso 2: qué secuencia ejecuta, qué parámetros usa (tiempos, velocidades, ángulos), cómo termina y cómo se testea con tiempo simulado. Explicar Strategy contra un `if/elif` gigante.
3. Generar:

   ```bash
   python .claude/skills/nuevo-nodo/render.py maniobra --nombre <nombre> --clase <Clase>
   ```

4. Reemplazar el ejemplo (mantener el auto detenido un tiempo) por la secuencia real, con los parámetros en el YAML de `bfmc_maneuvers`.
5. Conectar la estrategia al action server `ManeuverExecute`, que depende de `bfmc_interfaces`. Si ese paquete todavía no existe, avisarlo y proponer crearlo antes.

## Estándares

- **ROS2:** nombres de paquete `bfmc_<funcionalidad>`, archivos y topics en `snake_case`, `get_logger()` en lugar de `print`, QoS explícito, `destroy_node` y `try_shutdown` al cerrar, parámetros declarados.
- **Python:** PEP 8. El test de estilo (`ament_flake8`) forma parte de cada paquete y corre en el CI.
- **SOLID:** una responsabilidad por nodo y por clase; dependencias inyectadas (configuración y tiempo entran por argumento); la lógica no conoce a ROS.

## Si la persona discrepa

Antes de responder, abrir la fuente que respalda cada postura: la sección de `docs/ARQUITECTURA.md`, el código existente o la documentación de ROS2. Si la evidencia da la razón a la persona, cambiar la propuesta y decir por qué. Si no, mantener la postura, mostrar la evidencia y dejar que decida. Que insista no es evidencia.

## Al terminar

Recordar las convenciones del repositorio: rama `tipo/A2-NN-descripcion`, commits que empiezan con `A2-NN:` y PR con la plantilla completa. El merge lo hace una persona del equipo.
