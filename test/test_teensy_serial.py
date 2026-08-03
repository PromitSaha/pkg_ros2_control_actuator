from control_actuator.teensy_serial import TeensySerial


def test_build_moveall_command_converts_metres_to_millimetres():
    lengths = [0.0, 0.001, 0.01234, 0.1, 0.20199, 0.05]

    command = TeensySerial.build_moveall_command(lengths)

    assert command == 'MOVEALL 0.0 1.0 12.3 100.0 202.0 50.0\n'
