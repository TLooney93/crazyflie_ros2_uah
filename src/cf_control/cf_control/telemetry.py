"""Bounded, latest-value Crazyflie telemetry transport."""

import logging
import threading
import time
from dataclasses import dataclass
from queue import Empty, Full, Queue

from cflib.crazyflie.log import LogConfig


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class TelemetryVariable:
    """Describe one firmware log value and its radio representation."""

    name: str
    fetch_as: str
    required: bool = True


# Ten payload bytes per sample. These are the only onboard values used by the
# flight controller: XYZ checks Vicon/Kalman agreement before arming, battery
# gates arming, and thrust protects takeoff.
FLIGHT_TELEMETRY_VARIABLES = (
    TelemetryVariable('stateEstimate.x', 'FP16'),
    TelemetryVariable('stateEstimate.y', 'FP16'),
    TelemetryVariable('stateEstimate.z', 'FP16'),
    TelemetryVariable('pm.vbat', 'FP16'),
    TelemetryVariable('stabilizer.thrust', 'FP16', required=False),
)

WAND_TELEMETRY_VARIABLES = (
    TelemetryVariable('stateEstimate.x', 'FP16'),
    TelemetryVariable('stateEstimate.y', 'FP16'),
    TelemetryVariable('stateEstimate.z', 'FP16'),
    TelemetryVariable('stabilizer.roll', 'FP16'),
    TelemetryVariable('stabilizer.pitch', 'FP16'),
    TelemetryVariable('stabilizer.yaw', 'FP16'),
)


class TelemetryReader:
    """Deliver the newest sample from one compact firmware log block."""

    def __init__(
        self,
        scf,
        callback,
        period_ms=500,
        restart_timeout_s=3.0,
        restart_backoff_max_s=30.0,
        variables=FLIGHT_TELEMETRY_VARIABLES,
    ):
        """Configure a non-blocking latest-value telemetry reader."""
        if period_ms <= 0:
            raise ValueError('telemetry period must be positive')
        if restart_timeout_s <= 3.0 * period_ms / 1000.0:
            raise ValueError('restart timeout must exceed three periods')
        if restart_backoff_max_s < restart_timeout_s:
            raise ValueError('restart backoff maximum is too short')
        if not variables:
            raise ValueError('telemetry variables must not be empty')

        self.scf = scf
        self.callback = callback
        self.period_ms = int(period_ms)
        self.restart_timeout_s = float(restart_timeout_s)
        self.restart_backoff_max_s = float(restart_backoff_max_s)
        self.variables = tuple(variables)
        self._samples = Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._statistics_lock = threading.Lock()
        self._samples_received = 0
        self._samples_dropped = 0
        self._restart_attempts = 0
        self._last_sample_at = None
        self._last_error = None
        self.running = False
        self.thread = None

    def start(self):
        """Start the reader exactly once while its worker is alive."""
        if self.thread is not None and self.thread.is_alive():
            return
        self._discard_queued_sample()
        self._stop_event.clear()
        self.running = True
        self.thread = threading.Thread(
            target=self._run,
            name='CrazyflieTelemetryReader',
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        """Request shutdown and wait briefly when called externally."""
        self.running = False
        self._stop_event.set()
        if self.thread is None or self.thread is threading.current_thread():
            return
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            raise TimeoutError('Telemetry reader did not stop within 1 second')

    def consume_statistics(self):
        """Return and reset transport counters without blocking callbacks."""
        with self._statistics_lock:
            result = {
                'samples': self._samples_received,
                'dropped': self._samples_dropped,
                'restarts': self._restart_attempts,
                'last_sample_at': self._last_sample_at,
                'error': self._last_error,
            }
            self._samples_received = 0
            self._samples_dropped = 0
            self._restart_attempts = 0
            self._last_error = None
        return result

    def _on_log_data(self, timestamp, data, logconf):
        """Keep only the newest sample; never block cflib's receive thread."""
        if not self.running:
            return
        received_at = time.monotonic()
        sample = (received_at, data)
        try:
            self._samples.put_nowait(sample)
        except Full:
            try:
                self._samples.get_nowait()
            except Empty:
                pass
            try:
                self._samples.put_nowait(sample)
            except Full:
                return
            with self._statistics_lock:
                self._samples_dropped += 1
        with self._statistics_lock:
            self._samples_received += 1
            self._last_sample_at = received_at

    def _on_log_error(self, logconf, message):
        """Record firmware log errors without doing radio work in callbacks."""
        with self._statistics_lock:
            self._last_error = str(message)

    def _has_log_variable(self, name):
        """Return whether the connected firmware exposes a log variable."""
        return (
            self.scf.cf.log.toc.get_element_by_complete_name(name)
            is not None
        )

    def _create_log_config(self):
        """Create one compact block containing only requested values."""
        log_config = LogConfig(
            name='FlightHealth',
            period_in_ms=self.period_ms,
        )
        missing = []
        for variable in self.variables:
            if not self._has_log_variable(variable.name):
                if variable.required:
                    missing.append(variable.name)
                continue
            log_config.add_variable(variable.name, variable.fetch_as)
        if missing:
            raise RuntimeError(
                'firmware is missing required telemetry variables: '
                + ', '.join(missing)
            )
        return log_config

    def _run(self):
        try:
            self._run_with_recovery()
        except Exception as error:
            with self._statistics_lock:
                self._last_error = str(error)
            LOGGER.exception('Telemetry reader stopped unexpectedly')
        finally:
            self.running = False

    def _run_with_recovery(self):
        """Run one block and use bounded exponential recovery after silence."""
        log_config = self._create_log_config()
        self.scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self._on_log_data)
        log_config.error_cb.add_callback(self._on_log_error)
        try:
            try:
                log_config.start()
            except Exception as error:
                with self._statistics_lock:
                    self._last_error = f'initial start: {error}'
                LOGGER.warning('Telemetry log initial start failed: %s', error)
            last_sample_at = time.monotonic()
            next_restart_at = last_sample_at + self.restart_timeout_s
            backoff_s = self.restart_timeout_s

            while not self._stop_event.is_set():
                wait_s = max(
                    0.01,
                    min(0.25, next_restart_at - time.monotonic()),
                )
                try:
                    received_at, data = self._samples.get(timeout=wait_s)
                except Empty:
                    now = time.monotonic()
                    if now < next_restart_at:
                        continue
                    self._restart_log_block(log_config)
                    next_restart_at = now + backoff_s
                    backoff_s = min(
                        backoff_s * 2.0,
                        self.restart_backoff_max_s,
                    )
                    continue

                next_restart_at = received_at + self.restart_timeout_s
                backoff_s = self.restart_timeout_s
                try:
                    self.callback(data, received_at)
                except Exception as error:
                    with self._statistics_lock:
                        self._last_error = f'consumer callback: {error}'
                    LOGGER.exception('Telemetry consumer callback failed')
        finally:
            log_config.data_received_cb.remove_callback(self._on_log_data)
            log_config.error_cb.remove_callback(self._on_log_error)
            try:
                log_config.stop()
                log_config.delete()
            except Exception:
                LOGGER.exception('Failed to clean up telemetry log block')

    def _restart_log_block(self, log_config):
        """Perform one stop/start attempt, never faster than the backoff."""
        with self._statistics_lock:
            self._restart_attempts += 1
        LOGGER.warning(
            'Crazyflie telemetry silent for %.1f s; restarting log block',
            self.restart_timeout_s,
        )
        try:
            log_config.stop()
        except Exception:
            LOGGER.debug('Telemetry log stop failed during recovery')
        try:
            log_config.start()
        except Exception as error:
            with self._statistics_lock:
                self._last_error = f'restart: {error}'
            LOGGER.warning('Telemetry log restart failed: %s', error)

    def _discard_queued_sample(self):
        try:
            while True:
                self._samples.get_nowait()
        except Empty:
            return
