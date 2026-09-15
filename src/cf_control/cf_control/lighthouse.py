import time
import math
from collections import deque
from queue import Queue, Empty

from cflib.crazyflie.log import LogConfig

class LighthouseManager:

    def __init__(self, scf):
        self.scf = scf

    def verify(self, timeout_s=10.0):
        """
        Verify Lighthouse positioning appears usable.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError(
                'timeout_s must be finite and greater than zero'
            )
        deadline = time.monotonic() + timeout_s
        samples = Queue()
        history_x = deque(maxlen=10)
        history_y = deque(maxlen=10)
        history_z = deque(maxlen=10)
        histories = (history_x, history_y, history_z)
        last_received_at = None
        max_sample_age_s = 1.0
        variance_span_limit = 0.001

        log_config = LogConfig(
            name='LighthouseCheck',
            period_in_ms=500
        )

        log_config.add_variable('lighthouse.status', 'uint8_t')
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        def on_data(timestamp, data, logconf):
            samples.put((time.monotonic(), data))

        self.scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(on_data)

        try:
            log_config.start()

            while True:
                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    raise TimeoutError(
                        'Lighthouse verification timed out'
                    )

                try:
                    received_at, data = samples.get(
                        timeout=min(0.1, remaining)
                    )
                except Empty:
                    continue

                now = time.monotonic()

                if now >= deadline:
                    raise TimeoutError(
                        'Lighthouse verification timed out'
                    )

                if last_received_at is not None:
                    if received_at - last_received_at > max_sample_age_s:
                        for history in histories:
                            history.clear()

                last_received_at = received_at

                if (
                    now - received_at > max_sample_age_s
                    or data['lighthouse.status'] != 2
                ):
                    for history in histories:
                        history.clear()
                    continue

                variances = (
                    data['kalman.varPX'],
                    data['kalman.varPY'],
                    data['kalman.varPZ']
                )

                if not all(
                    math.isfinite(value) and value >= 0
                    for value in variances
                ):
                    for history in histories:
                        history.clear()
                    continue

                for history, value in zip(histories, variances):
                    history.append(value)

                if len(history_x) < 10:
                    continue

                if all(
                    max(history) - min(history) < variance_span_limit
                    for history in histories
                ):
                    return True

        finally:
            log_config.data_received_cb.remove_callback(on_data)
            try:
                log_config.stop()
            finally:
                log_config.delete()

    def reset_estimator(self):
        """
        Request an estimator reset while the drone is grounded.
        """
        self.scf.cf.param.set_value('kalman.resetEstimation', '1')

        try:
            time.sleep(0.1)
        finally:
            self.scf.cf.param.set_value('kalman.resetEstimation', '0')