"""Interactive R1 gesture-recognition trial logger."""

import csv
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, Thread
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


GESTURES = ('takeoff', 'land')


def summarize_trials(rows):
    """Return correct/total counts for every required gesture."""
    return {
        gesture: (
            sum(row['correct'] for row in rows
                if row['intended_gesture'] == gesture),
            sum(row['intended_gesture'] == gesture for row in rows),
        )
        for gesture in GESTURES
    }


class GestureTrialLogger(Node):
    """Capture the first detected gesture during an armed trial window."""

    def __init__(self):
        super().__init__('gesture_trial_logger')
        self.declare_parameter('operator', 'operator1')
        self.declare_parameter('trials_per_gesture', 30)
        self.declare_parameter('response_window_s', 2.0)
        self.declare_parameter('detected_topic', '/wand/detected_gesture')
        self.declare_parameter('output_file', '')
        self.operator = str(self.get_parameter('operator').value)
        self.trials_per_gesture = int(
            self.get_parameter('trials_per_gesture').value)
        self.response_window = float(
            self.get_parameter('response_window_s').value)
        output = str(self.get_parameter('output_file').value)
        if output:
            self.output_file = Path(output).expanduser()
        else:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_file = Path(
                f'output/test_results/r1_gestures_{self.operator}_{stamp}.csv')
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self._lock = Lock()
        self._event = Event()
        self._active = False
        self._detected = None
        self._detected_ns = None
        self.create_subscription(
            String,
            str(self.get_parameter('detected_topic').value),
            self._on_detection,
            10,
        )

    def _on_detection(self, message):
        with self._lock:
            if not self._active or self._detected is not None:
                return
            self._detected = message.data.strip().lower()
            self._detected_ns = time.monotonic_ns()
            self._event.set()

    def run_trial(self, trial_id, intended):
        """Open one response window and return its CSV-compatible row."""
        with self._lock:
            self._detected = None
            self._detected_ns = None
            self._active = True
            self._event.clear()
            started_ns = time.monotonic_ns()
        self._event.wait(self.response_window)
        with self._lock:
            self._active = False
            detected = self._detected or ''
            detected_ns = self._detected_ns
        latency_ms = (
            round((detected_ns - started_ns) / 1_000_000, 3)
            if detected_ns is not None else ''
        )
        return {
            'trial_id': trial_id,
            'operator': self.operator,
            'intended_gesture': intended,
            'detected_gesture': detected,
            'correct': detected == intended,
            'latency_ms': latency_ms,
            'recorded_at': datetime.now().isoformat(timespec='seconds'),
        }


def main(args=None):
    rclpy.init(args=args)
    node = GestureTrialLogger()
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    rows = []
    fieldnames = (
        'trial_id', 'operator', 'intended_gesture', 'detected_gesture',
        'correct', 'latency_ms', 'recorded_at')
    sequence = [
        gesture
        for _ in range(node.trials_per_gesture)
        for gesture in GESTURES
    ]
    try:
        print(f'R1 logger ready for {node.operator}.')
        print('Keep motor actuation disabled. Press Enter to arm each trial.')
        print('Hold IO1, perform the prompted motion, then release IO1.')
        for trial_id, intended in enumerate(sequence, start=1):
            input(
                f'[{trial_id}/{len(sequence)}] Ready for {intended.upper()} '
                '(Enter, or Ctrl+C to stop): ')
            row = node.run_trial(trial_id, intended)
            rows.append(row)
            detected = row['detected_gesture'] or 'nothing'
            verdict = 'PASS' if row['correct'] else 'FAIL'
            print(f'  {verdict}: detected {detected}')
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        print('\nTrial session stopped; saving completed trials.')
    finally:
        with node.output_file.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f'Results: {node.output_file}')
        for gesture, (correct, total) in summarize_trials(rows).items():
            status = 'PASS' if total == node.trials_per_gesture and correct >= 29 else 'INCOMPLETE/FAIL'
            print(f'{gesture}: {correct}/{total} - {status}')
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
