---
name: escribir-tests
description: Usar cuando haya que escribir, revisar o mejorar tests de un nodo, una lógica o un contrato del vehículo. Ayuda a decidir qué testear y de qué tipo, propone los casos antes de escribirlos y usa plantillas ejecutables con pytest para que un cambio no rompa lo que ya funciona sin avisar.
---

# Escribir tests

Guía a quien escribe tests para que entienda qué vale la pena verificar y por qué, en lugar de copiar una plantilla a ciegas. Un buen test avisa cuando un cambio rompe otra parte del sistema; ese es el objetivo.

## Reglas que valen en toda la skill

- **Proponer, no imponer.** Proponer qué testear y de qué tipo. La persona decide y puede refutar.
- **No ceder por complacencia.** Si hay un desacuerdo, verificar contra la documentación o el código antes de responder. Nunca cambiar de postura solo porque se insiste. Sostener lo correcto con evidencia.
- **Objetividad con datos.** Contrastar contra fuentes reales (el código, `docs/ARQUITECTURA.md`, los ADR, la documentación de pytest y ROS2), no contra suposiciones. Lo que no se pudo verificar se dice.
- **Explicar el porqué.** Cada test propuesto viene con el caso que cubre y por qué aporta.

## Antes de proponer nada

1. Leer el código a testear y las secciones de `docs/ARQUITECTURA.md` que lo describen (principios y estructura de un paquete).
2. Mirar los tests que el paquete ya tiene, para no duplicarlos ni contradecirlos.
3. Revisar si el código **se puede testear**: la lógica no debe leer el reloj, dormir ni tocar hardware. Si lo hace, proponer primero separarla (ver "Tiempo simulable"). Testear lógica acoplada obliga a tests lentos e inestables.

## Paso 1: clasificar

Cada caso corresponde a un tipo de test. La guía completa, con cuándo usar cada uno y qué no testear, está en [`guia-de-tipos-de-test.md`](guia-de-tipos-de-test.md).

| Qué se verifica | Tipo | Ejemplo ejecutable |
|---|---|---|
| Una regla de la lógica con datos conocidos (PID, conversiones, máquinas de estado, maniobras) | Unitario | [`test_unitario_tiempo_simulado.py`](ejemplos/test_unitario_tiempo_simulado.py) |
| Lo que un módulo le manda a otro respeta un formato o rango (comandos al ESP32, campos de un mensaje) | Contrato | [`test_contrato_comandos.py`](ejemplos/test_contrato_comandos.py) |
| Un nodo conecta bien sus mensajes con su lógica | Integración de nodo | [`test_nodo_ros.py`](ejemplos/test_nodo_ros.py) |
| Varias piezas juntas sobre datos reales de la pista | Integración con rosbag | Ver la guía; todavía no hay una grabación en el repositorio |

## Paso 2: proponer qué testear (obligatorio, antes de escribir)

Presentar la lista y esperar un sí explícito. No se escribe ningún test antes. Para cada test:

- **Nombre:** una frase que describe una regla del sistema, en español (por ejemplo, `test_al_cumplirse_el_tiempo_limite_se_vence`).
- **Tipo:** de la tabla anterior.
- **Caso que cubre:** qué entrada y qué resultado esperado.
- **Por qué aporta:** qué se rompería si ese test no existiera.

Incluir también qué **no** se va a testear y por qué (por ejemplo, detalles internos que cambian con cada refactor). Cerrar con: "¿Escribo estos tests? Podés agregar, quitar o cambiar cualquiera."

## Paso 3: escribir

- Ubicación: `ros2_ws/src/<paquete>/test/test_<tema>.py`.
- Partir de la plantilla del tipo correspondiente en `ejemplos/`. Son archivos reales que se pueden ejecutar tal cual.
- Estructura: preparar, actuar, verificar. Un concepto por test. Casos parecidos con `pytest.mark.parametrize`.
- El nombre de cada test describe una regla: leer la lista de nombres debe explicar cómo se comporta el sistema.
- Tests independientes entre sí, sin orden obligatorio y sin estado compartido.
- Sin `time.sleep`. Si hay que esperar mensajes reales, girar el executor con un tope de tiempo, como en el ejemplo de nodo.
- Estilo PEP 8. El paquete ya trae el test de `ament_flake8` y corre en el CI.

### Un test verifica, un script muestra

Un script que imprime valores obliga a que alguien mire la pantalla y decida si está bien. Un test **afirma** el resultado (`assert`) y falla solo. Si al escribir hay un `print`, falta un `assert`. Un test que nunca falla no protege nada.

### Tiempo simulable

La lógica recibe el instante actual (`now`) como argumento y no llama a `time.sleep` ni lee el reloj por su cuenta. Con eso, una maniobra de 8 segundos se verifica en milisegundos y el resultado no depende de la velocidad de la máquina. Si la lógica ya existe y lee el reloj, proponer el cambio y mostrar el costo (tocar la firma de un par de métodos) frente al beneficio (tests rápidos, estables y sin esperas).

## Paso 4: verificar

```bash
docker compose run --rm test
```

Mostrar el resultado. Después hacer una **prueba de sensibilidad**: romper la lógica a propósito (por ejemplo, cambiar un `>=` por `>`), confirmar que algún test falla y deshacer el cambio. Si nada falla, los tests no protegen esa regla y hay que reforzarlos. Decir en el reporte qué se verificó y qué no.

Lo que depende del auto real (puerto serial, cámara, latencias) no se puede verificar en un contenedor: dejarlo declarado como pendiente en el PR.

## Si la persona discrepa

Antes de responder, abrir la fuente que respalda cada postura: el código, la documentación de pytest o de ROS2, o la arquitectura del proyecto. Si la evidencia da la razón a la persona, cambiar la propuesta y decir por qué. Si no, mantener la postura, mostrar la evidencia y dejar que decida. Que insista no es evidencia.

## Al terminar

Recordar las convenciones del repositorio: rama `tipo/A2-NN-descripcion`, commits que empiezan con `A2-NN:` y PR con la plantilla completa. El merge lo hace una persona del equipo.
