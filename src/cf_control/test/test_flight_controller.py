import unittest

from cf_control.flight_controller import (
    FlightController,
    ground_effect_takeoff_boost,
    limit_horizontal_velocity,
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

    def test_horizontal_velocity_limit_preserves_direction(self):
        vx, vy = limit_horizontal_velocity(0.3, 0.4, 0.1)

        self.assertAlmostEqual(vx, 0.06)
        self.assertAlmostEqual(vy, 0.08)

    def test_horizontal_velocity_below_limit_is_unchanged(self):
        self.assertEqual(
            limit_horizontal_velocity(0.03, -0.04, 0.1),
            (0.03, -0.04),
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
