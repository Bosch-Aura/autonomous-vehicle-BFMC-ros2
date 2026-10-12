#!/usr/bin/env python3
"""Arma el esqueleto de un paquete ROS2 o de una maniobra a partir de las plantillas.

Uso, desde la raíz del repositorio:

    python .claude/skills/nuevo-nodo/render.py nodo \
        --paquete bfmc_control --nombre control --clase Control \
        --descripcion "Convierte el estado del carril en un ángulo de dirección."

    python .claude/skills/nuevo-nodo/render.py maniobra --nombre stop --clase Stop

Nunca sobrescribe archivos existentes. Solo usa la biblioteca estándar.
"""
import argparse
import re
import sys
from pathlib import Path

TEMPLATES = Path(__file__).resolve().parent / 'templates'

# plantilla -> destino dentro del paquete. Los marcadores {paquete} y {nombre}
# se reemplazan en la ruta; @PAQUETE@, @NOMBRE@, @CLASE@ y @DESCRIPCION@, en el contenido.
NODO = {
    'package.xml.tmpl': 'package.xml',
    'setup.py.tmpl': 'setup.py',
    'setup.cfg.tmpl': 'setup.cfg',
    'resource.tmpl': 'resource/{paquete}',
    'README.md.tmpl': 'README.md',
    'config/params.yaml.tmpl': 'config/params.yaml',
    'launch/node.launch.py.tmpl': 'launch/{nombre}.launch.py',
    'modulo/__init__.py.tmpl': '{paquete}/__init__.py',
    'modulo/logic.py.tmpl': '{paquete}/{nombre}_logic.py',
    'modulo/node.py.tmpl': '{paquete}/{nombre}_node.py',
    'test/test_logic.py.tmpl': 'test/test_{nombre}_logic.py',
    'test/test_node.py.tmpl': 'test/test_{nombre}_node.py',
    'test/test_flake8.py.tmpl': 'test/test_flake8.py',
}

# Archivos compartidos por todas las maniobras: se crean una sola vez.
MANIOBRA_COMPARTIDOS = {
    'maniobra/base.py.tmpl': '{paquete}/strategies/base.py',
    'maniobra/strategies_init.py.tmpl': '{paquete}/strategies/__init__.py',
}
MANIOBRA = {
    'maniobra/strategy.py.tmpl': '{paquete}/strategies/{nombre}.py',
    'maniobra/test_strategy.py.tmpl': 'test/test_{nombre}_strategy.py',
}

PAQUETE_MANIOBRAS = 'bfmc_maneuvers'
PATRON_PAQUETE = re.compile(r'^bfmc_[a-z][a-z0-9_]*$')
PATRON_NOMBRE = re.compile(r'^[a-z][a-z0-9_]*$')
PATRON_CLASE = re.compile(r'^[A-Z][A-Za-z0-9]*$')
CARACTERES_PROHIBIDOS = set('\'"<>&\n\r\\')


def fallar(mensaje):
    print('error: ' + mensaje, file=sys.stderr)
    sys.exit(1)


def validar(valor, patron, etiqueta, ejemplo):
    if not patron.match(valor):
        fallar('%s inválido: "%s". Ejemplo: %s' % (etiqueta, valor, ejemplo))


def renderizar(origen, destino, valores):
    contenido = (TEMPLATES / origen).read_text(encoding='utf-8')
    for marcador, valor in valores.items():
        contenido = contenido.replace(marcador, valor)
    destino.parent.mkdir(parents=True, exist_ok=True)
    destino.write_text(contenido, encoding='utf-8', newline='\n')
    print('creado: %s' % destino)


def destinos(mapa, base, rutas):
    return {origen: base / patron.format(**rutas) for origen, patron in mapa.items()}


def comando_nodo(args):
    validar(args.paquete, PATRON_PAQUETE, 'paquete', 'bfmc_control')
    validar(args.nombre, PATRON_NOMBRE, 'nombre', 'control')
    validar(args.clase, PATRON_CLASE, 'clase', 'Control')
    if not args.descripcion or CARACTERES_PROHIBIDOS & set(args.descripcion):
        fallar('la descripción es obligatoria y no puede tener comillas, <, >, & ni saltos de línea')

    base = Path(args.destino) / args.paquete
    if base.exists():
        fallar('%s ya existe; no se sobrescribe' % base)

    rutas = {'paquete': args.paquete, 'nombre': args.nombre}
    valores = {'@PAQUETE@': args.paquete, '@NOMBRE@': args.nombre,
               '@CLASE@': args.clase, '@DESCRIPCION@': args.descripcion}
    for origen, destino in destinos(NODO, base, rutas).items():
        renderizar(origen, destino, valores)


def comando_maniobra(args):
    validar(args.nombre, PATRON_NOMBRE, 'nombre', 'stop')
    validar(args.clase, PATRON_CLASE, 'clase', 'Stop')

    base = Path(args.destino) / PAQUETE_MANIOBRAS
    if not base.is_dir():
        fallar('%s no existe. Crealo primero con el comando "nodo" '
               '(--paquete %s)' % (base, PAQUETE_MANIOBRAS))

    rutas = {'paquete': PAQUETE_MANIOBRAS, 'nombre': args.nombre}
    valores = {'@PAQUETE@': PAQUETE_MANIOBRAS, '@NOMBRE@': args.nombre,
               '@CLASE@': args.clase, '@DESCRIPCION@': ''}

    propios = destinos(MANIOBRA, base, rutas)
    existentes = [str(d) for d in propios.values() if d.exists()]
    if existentes:
        fallar('ya existen: %s; no se sobrescribe' % ', '.join(existentes))

    for origen, destino in destinos(MANIOBRA_COMPARTIDOS, base, rutas).items():
        if destino.exists():
            print('ya existe, se conserva: %s' % destino)
        else:
            renderizar(origen, destino, valores)
    for origen, destino in propios.items():
        renderizar(origen, destino, valores)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='comando', required=True)

    nodo = sub.add_parser('nodo', help='crea un paquete ROS2 nuevo')
    nodo.add_argument('--paquete', required=True, help='p. ej. bfmc_control')
    nodo.add_argument('--nombre', required=True, help='nombre del nodo y del módulo, p. ej. control')
    nodo.add_argument('--clase', required=True, help='prefijo de las clases, p. ej. Control')
    nodo.add_argument('--descripcion', required=True, help='una frase, sin comillas')
    nodo.add_argument('--destino', default='ros2_ws/src')
    nodo.set_defaults(funcion=comando_nodo)

    maniobra = sub.add_parser('maniobra', help='agrega una maniobra a bfmc_maneuvers')
    maniobra.add_argument('--nombre', required=True, help='p. ej. stop')
    maniobra.add_argument('--clase', required=True, help='p. ej. Stop')
    maniobra.add_argument('--destino', default='ros2_ws/src')
    maniobra.set_defaults(funcion=comando_maniobra)

    args = parser.parse_args()
    args.funcion(args)


if __name__ == '__main__':
    main()
