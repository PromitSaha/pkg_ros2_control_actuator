class TeensySerial:
    """Send six actuator targets to a Teensy using its text protocol."""

    def __init__(self, port, baud_rate):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required; install the python3-serial package"
            ) from exc

        self._serial = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=1.0,
            write_timeout=1.0,
        )

    @staticmethod
    def build_moveall_command(lengths):
        """Convert extensions in metres to MOVEALL targets in millimetres."""
        targets = " ".join(f"{length * 1000.0:.1f}" for length in lengths)
        return f"MOVEALL {targets}\n"

    def send_moveall(self, lengths):
        command = self.build_moveall_command(lengths)
        self._serial.write(command.encode("ascii"))
        self._serial.flush()
        return command.rstrip()

    def close(self):
        if self._serial.is_open:
            self._serial.close()
