class CrazyflieInterface:
    """A command interface for one Crazyflie using an existing connection."""

    def __init__(self, scf):
        """Store the supplied SyncCrazyflie instance for sending commands."""
        self.scf = scf

    def set_parameter(self, name, value):
        """Request a firmware parameter update on the connected drone.

        Args:
            name: Full firmware parameter name in 'group.parameter' form.
            value: Numeric value or numeric string to assign.
        """
        self.scf.cf.param.set_value(name, str(value))

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Send one velocity setpoint in the world coordinate frame.

        Args:
            vx: Requested velocity along the world x-axis, in metres per second.
            vy: Requested velocity along the world y-axis, in metres per second.
            vz: Requested velocity along the world z-axis, in metres per second.
            yaw_rate: Requested turning rate, in degrees per second. Defaults to
                zero.

        The caller is responsible for limiting and repeatedly sending setpoints.
        """
        self.scf.cf.commander.send_velocity_world_setpoint(
            vx,
            vy,
            vz,
            yaw_rate
        )

    def stop(self):
        """Send a motor-stop setpoint, which does not perform a landing."""
        self.scf.cf.commander.send_stop_setpoint()
