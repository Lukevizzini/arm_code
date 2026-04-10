from arm_teleop.logitech_mapping import LogitechInterpreter


def test_joint3_uses_right_stick_vertical_axis_by_default() -> None:
    interpreter = LogitechInterpreter()

    output = interpreter.step(
        axes=[0.0, 0.0, 0.0, 0.0, -1.0],
        buttons=[],
        dt=0.1,
    )

    assert output.joint3_command > 0.0


def test_right_stick_horizontal_axis_drives_joint4_by_default() -> None:
    interpreter = LogitechInterpreter()

    output = interpreter.step(
        axes=[0.0, 0.0, 0.0, 1.0, 0.0],
        buttons=[],
        dt=0.1,
    )

    assert output.joint4_command > 0.0
