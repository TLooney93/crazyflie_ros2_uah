class CrazyflieInterface:

    def __init__(self, scf):
        self.scf = scf

    def set_parameter(self, name, value):
        self.scf.cf.param.set_value(name, str(value))

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        self.scf.cf.commander.send_velocity_world_setpoint(
            vx,
            vy,
            vz,
            yaw_rate
        )

    def stop(self):
        self.scf.cf.commander.send_stop_setpoint()