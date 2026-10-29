"""Run multiple hardware Crazyflie controllers through shared radios."""

from cf_control.config import DRONES, VICON_FORWARD_RATE_HZ
from cf_control.crazyflie_node import CrazyflieNode
from cf_control.wand_driver_node import WandDriverNode
import cflib.crtp
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


class HardwareSwarmRunner(Node):
    """Own the configuration for co-located hardware controller nodes."""

    def __init__(self):
        super().__init__('hardware_swarm_runner')

        self.declare_parameter('drone_ids', ['cf1', 'cf2'])
        self.declare_parameter('vicon_topics', ['', ''])
        self.declare_parameter('radio_uris', ['', ''])
        self.declare_parameter('positioning_source', 'vicon')
        self.declare_parameter(
            'vicon_forward_rate_hz',
            VICON_FORWARD_RATE_HZ,
        )
        self.declare_parameter('wand_control_enabled', False)
        self.declare_parameter('wand_mission_gestures_enabled', False)
        self.declare_parameter('external_targets_enabled', True)
        self.declare_parameter('flight_mode', 'external')
        self.declare_parameter('max_flight_time_s', 90.0)
        self.declare_parameter('mission_name', 'figure_eight')
        self.declare_parameter('physical_wand_enabled', False)
        self.declare_parameter('wand_id', 'cf2')
        self.declare_parameter('wand_uri', '')

    def create_controllers(self):
        """Create one ROS controller per drone in this process."""
        drone_ids = list(self.get_parameter('drone_ids').value)
        vicon_topics = list(self.get_parameter('vicon_topics').value)
        radio_uris = list(self.get_parameter('radio_uris').value)

        if not drone_ids or len(set(drone_ids)) != len(drone_ids):
            raise ValueError('drone_ids must be a non-empty unique list')
        if len(vicon_topics) != len(drone_ids):
            raise ValueError('vicon_topics must contain one item per drone')
        if len(radio_uris) != len(drone_ids):
            raise ValueError('radio_uris must contain one item per drone')
        radio_uris = [
            uri or DRONES.get(drone_id, '')
            for drone_id, uri in zip(drone_ids, radio_uris)
        ]
        if any(not uri for uri in radio_uris):
            raise ValueError('every hardware swarm member needs a radio URI')

        shared_values = {
            'simulation': False,
            'positioning_source': self.get_parameter(
                'positioning_source'
            ).value,
            'vicon_forward_rate_hz': self.get_parameter(
                'vicon_forward_rate_hz'
            ).value,
            'wand_control_enabled': self.get_parameter(
                'wand_control_enabled'
            ).value,
            'wand_mission_gestures_enabled': self.get_parameter(
                'wand_mission_gestures_enabled'
            ).value,
            'external_targets_enabled': self.get_parameter(
                'external_targets_enabled'
            ).value,
            'flight_mode': self.get_parameter('flight_mode').value,
            'max_flight_time_s': self.get_parameter(
                'max_flight_time_s'
            ).value,
            'mission_name': self.get_parameter('mission_name').value,
        }

        controllers = []
        for drone_id, vicon_topic, radio_uri in zip(
            drone_ids,
            vicon_topics,
            radio_uris,
        ):
            values = {
                **shared_values,
                'drone_id': drone_id,
                'vicon_pose_topic': vicon_topic,
                'radio_uri': radio_uri,
            }
            overrides = [
                Parameter(name, value=value)
                for name, value in values.items()
            ]
            controllers.append(
                CrazyflieNode(
                    node_name=f'{drone_id}_control',
                    parameter_overrides=overrides,
                )
            )

        return controllers

    def create_wand_driver(self):
        """Create the optional button/attitude wand in this radio process."""
        if not self.get_parameter('physical_wand_enabled').value:
            return None

        drone_ids = set(self.get_parameter('drone_ids').value)
        wand_id = self.get_parameter('wand_id').value
        if wand_id in drone_ids:
            raise ValueError(
                f'Physical wand {wand_id!r} cannot also be a flying drone'
            )
        overrides = [
            Parameter('wand_id', value=wand_id),
            Parameter(
                'wand_uri',
                value=self.get_parameter('wand_uri').value,
            ),
            Parameter('buttons_enabled', value=True),
        ]
        return WandDriverNode(parameter_overrides=overrides)


def main(args=None):
    """Connect and spin all hardware controllers in one shared process."""
    rclpy.init(args=args)
    runner = None
    controllers = []
    wand_driver = None
    executor = None

    try:
        cflib.crtp.init_drivers()
        runner = HardwareSwarmRunner()
        controllers = runner.create_controllers()
        wand_driver = runner.create_wand_driver()

        for controller in controllers:
            controller.connect()
        if wand_driver is not None:
            wand_driver.connect()

        executor = MultiThreadedExecutor(
            num_threads=max(2, len(controllers) + 1)
        )
        executor.add_node(runner)
        for controller in controllers:
            executor.add_node(controller)
        if wand_driver is not None:
            executor.add_node(wand_driver)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        for controller in reversed(controllers):
            try:
                controller.disconnect()
            except Exception as error:
                if runner is not None:
                    runner.get_logger().error(
                        f'Error disconnecting {controller.drone_id}: {error}'
                    )
            try:
                controller.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if wand_driver is not None:
            try:
                wand_driver.disconnect()
            except Exception as error:
                if runner is not None:
                    runner.get_logger().error(
                        f'Error disconnecting physical wand: {error}'
                    )
            try:
                wand_driver.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if runner is not None:
            try:
                runner.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
