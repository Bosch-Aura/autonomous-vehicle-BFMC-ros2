"""Arma líneas de texto a partir de los bytes que llegan por el puerto serial."""


class LineBuffer:
    """Acumula bytes y entrega líneas completas.

    Los bytes llegan en trozos arbitrarios: una línea puede venir partida en dos
    lecturas, o varias líneas en una sola. Una línea más larga que `max_line`
    se descarta completa, para que la basura no crezca sin límite.
    """

    def __init__(self, max_line: int = 256):
        if max_line <= 0:
            raise ValueError('max_line debe ser positivo')
        self._max_line = max_line
        self._buffer = bytearray()
        self._descartando = False

    def feed(self, datos: bytes):
        """Agrega bytes y devuelve la lista de líneas completas, sin el terminador."""
        lineas = []
        for byte in datos:
            if byte == 0x0A:  # '\n'
                if not self._descartando:
                    lineas.append(self._decodificar(self._buffer))
                self._buffer = bytearray()
                self._descartando = False
            elif len(self._buffer) < self._max_line:
                self._buffer.append(byte)
            else:
                self._descartando = True
                self._buffer = bytearray()
        return [linea for linea in lineas if linea]

    @staticmethod
    def _decodificar(buffer):
        # Los bytes que no son texto (ruido del cable) se reemplazan y la línea
        # no pasará la validación del protocolo.
        return buffer.decode('utf-8', errors='replace').strip()
