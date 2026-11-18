"""Lógica de la grabación (A2-158): nombres, ritmo de cuadros y escritura. Sin ROS.

Cada grabación es una carpeta:

    <fecha_hora>/
        frames/<instante_ns>.jpg     cuadros de la cámara, sin procesar
        events.jsonl                 una línea por mensaje: topic, recv_ns, stamp_ns, msg
        meta.json                    cuándo, con qué ritmo y cuántos cuadros
        params_<nodo>.yaml           `ros2 param dump` de los nodos (los copia el nodo)

Con eso se repite el detector y el control fuera del auto, con el mismo ritmo que tuvieron.
"""
from datetime import datetime
import json
import os
import re

CARPETA_DE_CUADROS = 'frames'
ARCHIVO_DE_EVENTOS = 'events.jsonl'
ARCHIVO_META = 'meta.json'


def nombre_de_grabacion(ahora: datetime, etiqueta: str = '') -> str:
    """'20261118_011500' o '20261118_011500_curva', con la etiqueta limpia de caracteres raros."""
    nombre = ahora.strftime('%Y%m%d_%H%M%S')
    limpia = re.sub(r'[^A-Za-z0-9_-]+', '-', etiqueta.strip()).strip('-')
    return '%s_%s' % (nombre, limpia) if limpia else nombre


def toca_guardar_cuadro(ultimo_ns, ahora_ns: int, max_fps: float) -> bool:
    """Si ya pasó el intervalo desde el último cuadro guardado (max_fps = 0: todos)."""
    if max_fps <= 0.0 or ultimo_ns is None:
        return True
    return (ahora_ns - ultimo_ns) * 1e-9 >= 1.0 / max_fps - 1e-6


class Sesion:
    """Una grabación en curso: escribe cuadros y eventos en su carpeta."""

    def __init__(self, ruta: str, inicio_iso: str, max_fps: float, jpeg_quality: int):
        self.ruta = ruta
        self.cuadros = 0
        self.eventos = 0
        self.bytes = 0
        self._meta = {'inicio': inicio_iso, 'max_fps': max_fps, 'jpeg_quality': jpeg_quality}
        os.makedirs(os.path.join(ruta, CARPETA_DE_CUADROS))
        self._eventos = open(os.path.join(ruta, ARCHIVO_DE_EVENTOS), 'w', encoding='utf-8')
        self._escribir_meta()

    def _escribir_meta(self, extra=None):
        datos = dict(self._meta, cuadros=self.cuadros, eventos=self.eventos, bytes=self.bytes)
        datos.update(extra or {})
        with open(os.path.join(self.ruta, ARCHIVO_META), 'w', encoding='utf-8') as archivo:
            json.dump(datos, archivo, indent=2, ensure_ascii=False)

    def guardar_cuadro(self, instante_ns: int, jpeg: bytes) -> None:
        ruta = os.path.join(self.ruta, CARPETA_DE_CUADROS, '%d.jpg' % instante_ns)
        with open(ruta, 'wb') as archivo:
            archivo.write(jpeg)
        self.cuadros += 1
        self.bytes += len(jpeg)

    def guardar_evento(self, topic: str, recv_ns: int, stamp_ns, mensaje: dict) -> None:
        linea = {'topic': topic, 'recv_ns': recv_ns, 'stamp_ns': stamp_ns, 'msg': mensaje}
        self._eventos.write(json.dumps(linea, ensure_ascii=False) + '\n')
        self.eventos += 1
        if self.eventos % 20 == 0:
            self._eventos.flush()

    def cerrar(self, fin_iso: str, motivo: str) -> None:
        self._eventos.close()
        self._escribir_meta({'fin': fin_iso, 'motivo_fin': motivo})
