import threading
from queue import Queue, Empty, Full
from cflib.crazyflie.log import LogConfig



class TelemetryReader:

    def __init__(self, scf, callback, period_ms=100):
        self.scf = scf
        self.callback = callback
        self.period_ms = period_ms
        self._samples = Queue(maxsize=1)

        self.running = False
        self.thread = None

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return

        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True
        )

        self.thread.start()

    def stop(self):
        """Request shutdown and wait up to one second when called externally.

        Calls from the reader thread only request shutdown to avoid joining
        the current thread. Raise TimeoutError if an external wait expires
        while the reader is still alive.
        """
        self.running = False

        if self.thread is None:
            return

        if self.thread is threading.current_thread():
            return

        self.thread.join(timeout=1.0)

        if self.thread.is_alive():
            raise TimeoutError("Telemetry reader did not stop within 1 second")

    def _on_log_data(self, timestamp, data, logconf):
        while self.running:
            try:
                self._samples.put_nowait(data)
                return
            except Full:
                try:
                    self._samples.get_nowait()
                except Empty:
                    pass

    def _run(self):
        log_config = LogConfig(
            name='State',
            period_in_ms=self.period_ms
        )

        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')
        self.scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self._on_log_data)

        try:
            log_config.start()

            while self.running:
                try:
                    data = self._samples.get(timeout=0.1)
                except Empty:
                    # Check the running flag again when no sample arrives.
                    continue

                if self.running:
                    self.callback(data)

        finally:
            # Clean up when the loop exits or the callback raises an exception.
            self.running = False
            log_config.data_received_cb.remove_callback(self._on_log_data)
            log_config.stop()
            log_config.delete()


