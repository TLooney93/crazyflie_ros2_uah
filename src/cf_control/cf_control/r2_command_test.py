"""Dry ROS integration harness for requirement R2."""

import csv
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, Thread
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


COMMANDS = (
    ('takeoff', 'takeoff'),
    ('land', 'land'),
    ('emergency_stop', 'emergency_stop'),
    ('invalid_command', ''),
)


def command_passed(expected, received):
    """Return true when a command caused exactly its expected service."""
    return received == expected


class R2CommandHarness(Node):
    """Provide fake services and observe coordinator service requests."""

    def __init__(self):
        super().__init__('r2_command_test')
        self.declare_parameter('trials_per_command', 30)
        self.declare_parameter('command_timeout_s', 1.0)
        self.declare_parameter('invalid_window_s', 0.25)
        self.declare_parameter('output_file', '')
        self.trials_per_command = int(
            self.get_parameter('trials_per_command').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout_s').value)
        self.invalid_window = float(
            self.get_parameter('invalid_window_s').value)
        output = str(self.get_parameter('output_file').value)
        if output:
            self.output_file = Path(output).expanduser()
        else:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_file = Path(
                f'output/test_results/r2_commands_{stamp}.csv')
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.publisher = self.create_publisher(String, '/wand/gesture', 10)
        self._lock = Lock()
        self._event = Event()
        self._received = ''
        self._received_ns = None
        self._test_services = [
            self.create_service(
                Trigger, f'/r2test/{name}', self._callback(name))
            for name in ('takeoff', 'land', 'emergency_stop')
        ]

    def _callback(self, name):
        def callback(request, response):
            del request
            with self._lock:
                if not self._received:
                    self._received = name
                    self._received_ns = time.monotonic_ns()
                    self._event.set()
            response.success = True
            response.message = f'R2 fake {name} accepted'
            return response
        return callback

    def publish(self, value):
        message = String()
        message.data = value
        self.publisher.publish(message)

    def run_trial(self, trial_id, command, expected):
        with self._lock:
            self._received = ''
            self._received_ns = None
            self._event.clear()
            started_ns = time.monotonic_ns()
        if command == 'takeoff':
            self.publish('enable')
            time.sleep(0.03)
            started_ns = time.monotonic_ns()
        self.publish(command)
        self._event.wait(
            self.command_timeout if expected else self.invalid_window)
        with self._lock:
            received = self._received
            received_ns = self._received_ns
        latency_ms = (
            round((received_ns - started_ns) / 1_000_000, 3)
            if received_ns is not None else '')
        return {
            'trial_id': trial_id,
            'input': command,
            'expected_service': expected,
            'received_service': received,
            'correct': command_passed(expected, received),
            'latency_ms': latency_ms,
            'recorded_at': datetime.now().isoformat(timespec='seconds'),
        }


def main(args=None):
    rclpy.init(args=args)
    node = R2CommandHarness()
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    rows = []
    fields = (
        'trial_id', 'input', 'expected_service', 'received_service',
        'correct', 'latency_ms', 'recorded_at')
    try:
        print('R2 dry test waiting for /wand/gesture subscriber...')
        deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'No coordinator subscribed. Start wand_coordinator with '
                    'drone_id:=r2test.')
            time.sleep(0.1)
        time.sleep(1.0)
        total = node.trials_per_command * len(COMMANDS)
        trial_id = 0
        for _ in range(node.trials_per_command):
            for command, expected in COMMANDS:
                trial_id += 1
                row = node.run_trial(trial_id, command, expected)
                rows.append(row)
                verdict = 'PASS' if row['correct'] else 'FAIL'
                received = row['received_service'] or 'nothing'
                print(
                    f'[{trial_id}/{total}] {verdict}: {command} -> {received}')
    finally:
        with node.output_file.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        print(f'Results: {node.output_file}')
        for command, _ in COMMANDS:
            selected = [row for row in rows if row['input'] == command]
            correct = sum(row['correct'] for row in selected)
            status = (
                'PASS' if len(selected) == node.trials_per_command
                and correct == node.trials_per_command else 'FAIL')
            print(f'{command}: {correct}/{len(selected)} - {status}')
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
