import threading

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


class TelemetryReader:

    def __init__(self, scf, callback, period_ms=100):
        self.scf = scf
        self.callback = callback
        self.period_ms = period_ms

        self.running = False
        self.thread = None

    def start(self):
        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True
        )

        self.thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        log_config = LogConfig(
            name='State',
            period_in_ms=self.period_ms
        )

        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')

        with SyncLogger(self.scf, log_config) as logger:

            for timestamp, data, logconf in logger:

                if not self.running:
                    break

                self.callback(data)