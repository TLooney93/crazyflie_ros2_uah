import unittest

from cf_control.config import (
    CONTROL_RATE_HZ,
    TAKEOFF_ASSIST_GRACE_S,
    TAKEOFF_ASSIST_RAMP_DOWN_S,
    TAKEOFF_BOOST_END_HEIGHT_M,
    TAKEOFF_BOOST_M_S,
    TAKEOFF_BOOST_TAPER_START_HEIGHT_M,
    TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_HEIGHT_TOLERANCE_M,
    TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
    TAKEOFF_NEAR_FLOOR_HEIGHT_M,
    TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_PROGRESS_TIMEOUT_S,
)
from cf_control.flight_controller import FlightController
from cf_control.takeoff import (
    ground_effect_takeoff_boost,
    limit_horizontal_velocity,
    takeoff_horizontal_velocity_limit,
)


class FlightControllerTest(unittest.TestCase):

    def test_command_refresh_rate_supports_smooth_hardware_control(self):
        self.assertGreaterEqual(CONTROL_RATE_HZ, 50.0)

    def test_heavy_airframe_takeoff_profile_has_bounded_authority(self):
        self.assertGreater(TAKEOFF_BOOST_M_S, 0.0)
        self.assertGreater(TAKEOFF_MAX_VERTICAL_VELOCITY_M_S, 0.0)
        self.assertGreater(TAKEOFF_PROGRESS_TIMEOUT_S, 0.0)
        self.assertLess(
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
        )

    def test_ground_effect_boost_only_applies_below_cutoff(self):
        self.assertAlmostEqual(
            ground_effect_takeoff_boost(
                0.05,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
            ),
            TAKEOFF_BOOST_M_S,
        )
        self.assertEqual(
            ground_effect_takeoff_boost(
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
            ),
            0.0,
        )

    def test_ground_effect_boost_tapers_instead_of_stepping(self):
        midpoint = (
            TAKEOFF_BOOST_TAPER_START_HEIGHT_M
            + TAKEOFF_BOOST_END_HEIGHT_M
        ) / 2.0

        self.assertAlmostEqual(
            ground_effect_takeoff_boost(
                midpoint,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
                TAKEOFF_BOOST_TAPER_START_HEIGHT_M,
            ),
            TAKEOFF_BOOST_M_S / 2.0,
        )

    def test_boost_is_zero_before_standard_hover_transition(self):
        earliest_hover_height = 0.30 - TAKEOFF_HEIGHT_TOLERANCE_M

        self.assertAlmostEqual(
            ground_effect_takeoff_boost(
                earliest_hover_height,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
                TAKEOFF_BOOST_TAPER_START_HEIGHT_M,
            ),
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

    def test_takeoff_uses_stricter_horizontal_limit_near_floor(self):
        near_floor_limit = takeoff_horizontal_velocity_limit(
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M - 0.01,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
        )
        airborne_limit = takeoff_horizontal_velocity_limit(
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
        )

        self.assertEqual(
            near_floor_limit,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
        )
        self.assertEqual(
            airborne_limit,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
        )

    def test_takeoff_grace_uses_loose_horizontal_limit(self):
        limit = takeoff_horizontal_velocity_limit(
            TAKEOFF_ASSIST_GRACE_S - 0.01,
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
            0.0,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
        )

        self.assertEqual(
            limit,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
        )

    def test_horizontal_grace_limit_ramps_down_without_step(self):
        boundary = takeoff_horizontal_velocity_limit(
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_ASSIST_RAMP_DOWN_S,
        )
        midpoint = takeoff_horizontal_velocity_limit(
            TAKEOFF_ASSIST_GRACE_S + TAKEOFF_ASSIST_RAMP_DOWN_S / 2.0,
            TAKEOFF_ASSIST_GRACE_S,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_HEIGHT_M,
            TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
            TAKEOFF_ASSIST_RAMP_DOWN_S,
        )

        self.assertEqual(
            boundary,
            TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
        )
        self.assertAlmostEqual(
            midpoint,
            (
                TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S
                + TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S
            ) / 2.0,
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

    def test_integral_correction_is_bounded_and_resets_for_new_target(self):
        controller = FlightController(
            kp=0.5,
            ki=0.2,
            max_velocity=0.3,
            integral_max_velocity=0.06,
        )
        controller.set_target(0.0, 0.0, 1.0)

        for _ in range(100):
            velocity = controller.update((0.0, 0.0, 0.9), dt_s=0.05)

        self.assertAlmostEqual(velocity[2], 0.11)
        controller.set_target(0.0, 0.0, 0.9)
        self.assertEqual(
            controller.update((0.0, 0.0, 0.9), dt_s=0.05),
            (0.0, 0.0, 0.0),
        )

    def test_horizontal_target_change_preserves_vertical_integral(self):
        controller = FlightController(
            kp=0.5,
            ki=0.2,
            max_velocity=0.3,
            integral_max_velocity=0.06,
        )
        controller.set_target(0.0, 0.0, 1.0)
        for _ in range(20):
            controller.update((0.0, 0.0, 0.9), dt_s=0.05)

        vertical_integral = controller.integral_error[2]
        controller.set_target(0.1, -0.1, 1.0)

        self.assertEqual(controller.integral_error[0], 0.0)
        self.assertEqual(controller.integral_error[1], 0.0)
        self.assertEqual(controller.integral_error[2], vertical_integral)


if __name__ == '__main__':
    unittest.main()
