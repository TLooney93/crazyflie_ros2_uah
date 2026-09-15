import unittest

from cf_control.flight_controller import FlightController


class FlightControllerTest(unittest.TestCase):

    def test_no_target_produces_zero_velocity(self):
        controller = FlightController(kp=0.5, max_velocity=0.3)

        self.assertEqual(
            controller.update((1.0, 2.0, 3.0)),
            (0.0, 0.0, 0.0),
        )

    def test_position_error_produces_proportional_velocity(self):
        controller = FlightController(kp=0.5, max_velocity=0.3)
        controller.set_target(0.2, -0.4, 0.1)

        self.assertEqual(
            controller.update((0.0, 0.0, 0.0)),
            (0.1, -0.2, 0.05),
        )

    def test_velocity_is_limited_on_each_axis(self):
        controller = FlightController(kp=1.0, max_velocity=0.3)
        controller.set_target(2.0, -2.0, 0.3)

        self.assertEqual(
            controller.update((0.0, 0.0, 0.0)),
            (0.3, -0.3, 0.3),
        )


if __name__ == '__main__':
    unittest.main()
