"""Bounded newline framing for STM32 JSON sent over a timed serial port."""


class JsonLineFramer:
    def __init__(self, max_line_bytes=2048):
        self.max_line_bytes = max_line_bytes
        self._buffer = bytearray()
        self._discarding = False
        self.dropped_lines = 0

    def feed(self, chunk):
        """Return complete UTF-8 lines; keep an incomplete line for the next read."""
        lines = []
        for byte in chunk:
            if byte == 10:
                if self._discarding:
                    self._discarding = False
                elif self._buffer:
                    try:
                        lines.append(self._buffer.rstrip(b"\r").decode("utf-8"))
                    except UnicodeDecodeError:
                        self.dropped_lines += 1
                self._buffer.clear()
            elif not self._discarding:
                if len(self._buffer) < self.max_line_bytes:
                    self._buffer.append(byte)
                else:
                    self._buffer.clear()
                    self._discarding = True
                    self.dropped_lines += 1
        return lines
