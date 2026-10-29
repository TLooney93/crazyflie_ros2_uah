import unittest

from cf_control.flight_controller import (
    FlightController,
    ground_effect_takeoff_boost,
)


class FlightControllerTest(unittest.TestCase):

    def test_ground_effect_boost_only_applies_below_cutoff(self):
        self.assertEqual(
            ground_effect_takeoff_boost(0.05, 0.15, 0.08),
            0.08,
        )
        self.assertEqual(
            ground_effect_takeoff_boost(0.15, 0.15, 0.08),
            0.0,
        )

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
