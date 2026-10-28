from cf_control.r2_command_test import command_passed


def test_command_passed_requires_exact_expected_service():
    assert command_passed('takeoff', 'takeoff')
    assert not command_passed('takeoff', 'land')
    assert command_passed('', '')
    assert not command_passed('', 'emergency_stop')
