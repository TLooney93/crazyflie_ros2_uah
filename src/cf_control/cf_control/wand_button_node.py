"""Publish wand gestures from button states exposed by Crazyflie firmware."""

import cflib.crtp
import rclpy
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from cf_control.config import CACHE_DIR, DRONES, WAND_GESTURE_TOPIC


class ButtonEdges:
    """Convert sampled button states into one event per press."""

    def __init__(self):
        self.previous = {}

    def pressed(self, name, value):
        """Return true only on a zero-to-one transition."""
        current = bool(value)
        was_pressed = self.previous.get(name, False)
        self.previous[name] = current
        return current and not was_pressed


class WandButtonNode(Node):
    """Read custom firmware log variables and publish normalized gestures."""

    BUTTONS = {
        'wand.enable': 'enable',
        'wand.disable': 'disable',
        'wand.takeoff': 'takeoff',
        'wand.land': 'land',
        'wand.stop': 'emergency_stop',
    }

    def __init__(self):
        super().__init__('wand_button_node')
        self.declare_parameter('wand_id', 'cf2')
        self.wand_id = self.get_parameter('wand_id').value
        if self.wand_id not in DRONES:
            raise ValueError(f'Unknown wand ID: {self.wand_id}')
        self.uri = DRONES[self.wand_id]
        self.publisher = self.create_publisher(String, WAND_GESTURE_TOPIC, 10)
        self.edges = ButtonEdges()
        self.scf = None
        self.log_config = None

    def _on_buttons(self, timestamp, data, log_config):
        for variable, gesture in self.BUTTONS.items():
            if self.edges.pressed(variable, data.get(variable, 0)):
                message = String()
                message.data = gesture
                self.publisher.publish(message)
                self.get_logger().warning(f'Button gesture: {gesture}')

    def _on_log_error(self, log_config, message):
        self.get_logger().error(f'Button log error: {message}')

    def connect(self):
        """Connect to the wand and start its button log block."""
        self.scf = SyncCrazyflie(
            self.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))
        )
        self.scf.open_link()
        self.log_config = LogConfig(name='WandButtons', period_in_ms=50)
        for variable in self.BUTTONS:
            self.log_config.add_variable(variable, 'uint8_t')
        self.scf.cf.log.add_config(self.log_config)
        self.log_config.data_received_cb.add_callback(self._on_buttons)
        self.log_config.error_cb.add_callback(self._on_log_error)
        self.log_config.start()
        self.get_logger().info(f'Connected to button wand: {self.wand_id}')

    def disconnect(self):
        """Stop logging and close the radio link."""
        try:
            if self.log_config is not None:
                self.log_config.stop()
        finally:
            if self.scf is not None:
                self.scf.close_link()


def main(args=None):
    """Run the physical button wand driver."""
    rclpy.init(args=args)
    node = None
    try:
        cflib.crtp.init_drivers()
        node = WandButtonNode()
        node.connect()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
