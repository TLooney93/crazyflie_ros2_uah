import math
from threading import Condition, Lock, Thread
import time

from cf_control.config import (
    ARM_CONFIRM_TIMEOUT_S,
    RADIO_COMMAND_RATE_HZ,
    RADIO_COMMAND_TIMEOUT_S,
)


class RadioCommandDispatcher:
    """
    Transmit one deterministic, latest-value frame for a shared radio.

    A stock Crazyradio must address each Crazyflie separately. A dispatch
    frame therefore consists of one velocity packet for every owner with a
    fresh command, followed by at most one pending external-position packet
    per owner. Keeping the group snapshot and ordering in one worker avoids
    three independent controller threads competing for the radio.
    """

    def __init__(
        self,
        rate_hz=RADIO_COMMAND_RATE_HZ,
        command_timeout_s=RADIO_COMMAND_TIMEOUT_S,
    ):
        if not math.isfinite(rate_hz) or rate_hz < 50.0:
            raise ValueError('shared radio command rate must be at least 50 Hz')
        if (
            not math.isfinite(command_timeout_s)
            or command_timeout_s <= 0.0
        ):
            raise ValueError('command timeout must be positive and finite')
        self.rate_hz = float(rate_hz)
        self.command_timeout_s = float(command_timeout_s)
        self._period_s = 1.0 / self.rate_hz
        self._condition = Condition()
        self._pending = {}
        self._worker = None
        self._running = False

    def start(self):
        """Start the single priority worker if it is not already running."""
        with self._condition:
            if self._worker is not None and self._worker.is_alive():
                return
            self._running = True
            self._worker = Thread(
                target=self._run,
                name='CrazyflieSharedRadioDispatcher',
                daemon=True,
            )
            self._worker.start()

    def register(self, owner):
        """Register one interface for bounded latest-value scheduling."""
        with self._condition:
            self._pending.setdefault(
                owner,
                {'velocity': None, 'external_position': None},
            )

    def submit_velocity(self, owner, command):
        """Replace the owner's pending velocity with the newest command."""
        return self._submit(owner, 'velocity', command)

    def submit_external_position(self, owner, command):
        """Replace the owner's pending external pose with the newest pose."""
        return self._submit(owner, 'external_position', command)

    def _submit(self, owner, kind, command):
        with self._condition:
            slots = self._pending.get(owner)
            if not self._running or slots is None:
                return False
            slots[kind] = (time.monotonic(), command)
            self._condition.notify()
            return True

    def discard(self, owner):
        """Discard all pending high-rate commands for one interface."""
        with self._condition:
            slots = self._pending.get(owner)
            if slots is not None:
                slots['velocity'] = None
                slots['external_position'] = None

    def unregister(self, owner):
        """Remove an interface after invalidating its command generation."""
        with self._condition:
            self._pending.pop(owner, None)

    def shutdown(self):
        """Stop scheduling and discard all pending high-rate commands."""
        with self._condition:
            self._running = False
            for slots in self._pending.values():
                slots['velocity'] = None
                slots['external_position'] = None
            self._condition.notify_all()
            worker = self._worker
        if worker is not None:
            worker.join(timeout=7.0)
            if worker.is_alive():
                raise TimeoutError('Shared radio dispatcher did not stop')
        self._worker = None

    def _run(self):
        """Send a group snapshot on a fixed clock without catch-up bursts."""
        next_frame_at = time.monotonic() + self._period_s
        while True:
            with self._condition:
                while self._running:
                    frame_at = time.monotonic()
                    remaining_s = next_frame_at - frame_at
                    if remaining_s <= 0.0:
                        break
                    self._condition.wait(timeout=remaining_s)
                else:
                    return

                velocities = []
                external_positions = []
                stale_owners = []
                for owner, slots in self._pending.items():
                    velocity = slots['velocity']
                    if velocity is not None:
                        updated_at, command = velocity
                        if frame_at - updated_at <= self.command_timeout_s:
                            velocities.append((owner, command))
                        else:
                            slots['velocity'] = None
                            stale_owners.append(owner)
                    if slots['external_position'] is not None:
                        _, command = slots['external_position']
                        external_positions.append(
                            (owner, command)
                        )
                        slots['external_position'] = None

            for owner in stale_owners:
                owner._record_stale_velocity()
            for owner, command in velocities:
                self._dispatch(owner, owner._send_velocity_if_current, command)
            for owner, command in external_positions:
                self._dispatch(
                    owner,
                    owner._send_external_position_if_current,
                    command,
                )

            # Preserve the common frame clock when work completes on time. If
            # radio I/O overruns one or more frames, skip them instead of
            # emitting a burst of delayed setpoints.
            finished_at = time.monotonic()
            frames_elapsed = max(
                1,
                math.floor(
                    (finished_at - next_frame_at) / self._period_s
                ) + 1,
            )
            if frames_elapsed > 1:
                delay_s = finished_at - next_frame_at
                for owner, _command in velocities:
                    owner._record_dispatch_overrun(
                        frames_elapsed - 1,
                        delay_s,
                    )
            next_frame_at += frames_elapsed * self._period_s

    @staticmethod
    def _dispatch(owner, callback, command):
        try:
            callback(*command)
        except Exception as error:
            owner._record_send_error(error)


class CrazyflieInterface:
    """A command interface for one Crazyflie using an existing connection."""

    def __init__(self, scf, dispatcher=None):
        """Store the supplied SyncCrazyflie instance for sending commands."""
        self.scf = scf
        self._state_lock = Lock()
        self._radio_lock = Lock()
        self._dispatcher = dispatcher or RadioCommandDispatcher()
        self._owns_dispatcher = dispatcher is None
        self._started = False
        self._command_generation = 0
        self._sent_velocity = 0
        self._sent_external_position = 0
        self._stale_velocity_drops = 0
        self._dispatch_frames_skipped = 0
        self._max_dispatch_delay_s = 0.0
        self._last_velocity_sent_at = None
        self._max_velocity_gap_s = 0.0
        self._last_send_error = None
        self._radio_uplink_rate_hz = None
        self._radio_uplink_payload_fraction = None
        self._radio_link_quality = None
        self._radio_statistics_received_at = None
        self._link_statistics_callbacks = []

    def start(self):
        """Register this link with its shared priority dispatcher."""
        with self._state_lock:
            if self._started:
                return
            self._started = True
        self._register_link_statistics_callbacks()
        self._dispatcher.register(self)
        self._dispatcher.start()

    def shutdown(self):
        """Invalidate queued updates and unregister this radio link."""
        with self._state_lock:
            self._started = False
            self._command_generation += 1
            self._last_velocity_sent_at = None
        self._dispatcher.unregister(self)
        self._remove_link_statistics_callbacks()
        if self._owns_dispatcher:
            self._dispatcher.shutdown()

    def _register_link_statistics_callbacks(self):
        """Subscribe to cflib's measured, over-the-air link statistics."""
        statistics = getattr(self.scf.cf, 'link_statistics', None)
        if statistics is None or self._link_statistics_callbacks:
            return
        subscriptions = (
            ('uplink_rate_updated', self._update_radio_uplink_rate),
            (
                'uplink_congestion_updated',
                self._update_radio_uplink_payload_fraction,
            ),
            ('link_quality_updated', self._update_radio_link_quality),
        )
        for caller_name, callback in subscriptions:
            caller = getattr(statistics, caller_name, None)
            if caller is None or not hasattr(caller, 'add_callback'):
                continue
            caller.add_callback(callback)
            self._link_statistics_callbacks.append((caller, callback))

    def _remove_link_statistics_callbacks(self):
        """Detach callbacks before the interface and its ROS node disappear."""
        callbacks = self._link_statistics_callbacks
        self._link_statistics_callbacks = []
        for caller, callback in callbacks:
            try:
                caller.remove_callback(callback)
            except ValueError:
                pass

    def _update_radio_uplink_rate(self, rate_hz):
        """Store cflib's latest measured packet rate for this radio link."""
        with self._state_lock:
            self._radio_uplink_rate_hz = float(rate_hz)
            self._radio_statistics_received_at = time.monotonic()

    def _update_radio_uplink_payload_fraction(self, fraction):
        """Store the fraction of uplink packets carrying non-null payloads."""
        with self._state_lock:
            self._radio_uplink_payload_fraction = float(fraction)
            self._radio_statistics_received_at = time.monotonic()

    def _update_radio_link_quality(self, quality):
        """Store cflib's retry-derived link quality percentage."""
        with self._state_lock:
            self._radio_link_quality = float(quality)
            self._radio_statistics_received_at = time.monotonic()

    def _clear_pending(self):
        """Prevent an old setpoint from being sent after a safety command."""
        with self._state_lock:
            self._command_generation += 1
            self._last_velocity_sent_at = None
        self._dispatcher.discard(self)

    def set_parameter(self, name, value):
        """Set firmware parameter ``name`` to the numeric ``value``."""
        self.scf.cf.param.set_value(name, str(value))

    def arm(self, timeout_s=ARM_CONFIRM_TIMEOUT_S):
        """Arm and return only after the firmware confirms it can fly."""
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError('arming confirmation timeout must be positive')

        supervisor = self.scf.cf.supervisor
        with self._radio_lock:
            supervisor.send_arming_request(True)

        deadline = time.monotonic() + timeout_s
        last_states = []
        blocking_states = {'Is tumbled', 'Is locked', 'Is crashed'}
        while True:
            with self._radio_lock:
                last_states = list(supervisor.read_state_list())

            blockers = blocking_states.intersection(last_states)
            if blockers:
                detail = ', '.join(sorted(blockers))
                raise RuntimeError(
                    f'firmware supervisor rejected arming: {detail}'
                )
            if 'Is armed' in last_states and 'Can fly' in last_states:
                return tuple(last_states)

            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                break
            time.sleep(min(0.05, remaining_s))

        state_text = ', '.join(last_states) or 'no states reported'
        raise TimeoutError(
            'firmware did not confirm Is armed and Can fly within '
            f'{timeout_s:.1f} s; supervisor states: {state_text}'
        )

    def disarm(self):
        """Request that the Crazyflie disarm its motors."""
        self._clear_pending()
        with self._radio_lock:
            self.scf.cf.supervisor.send_arming_request(False)

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Queue a bounded world-frame velocity setpoint for transmission."""
        with self._state_lock:
            started = self._started
            command = (
                self._command_generation,
                vx,
                vy,
                vz,
                yaw_rate,
            )
        if started and self._dispatcher.submit_velocity(self, command):
            return
        self._send_velocity_now(vx, vy, vz, yaw_rate)

    def _send_velocity_now(self, vx, vy, vz, yaw_rate):
        """Send a velocity command from the dedicated worker."""
        with self._radio_lock:
            self.scf.cf.commander.send_velocity_world_setpoint(
                vx,
                vy,
                vz,
                yaw_rate,
            )

    def _send_velocity_if_current(self, generation, vx, vy, vz, yaw_rate):
        """Send a queued velocity only if no safety command superseded it."""
        with self._radio_lock:
            with self._state_lock:
                if generation != self._command_generation:
                    return
            self.scf.cf.commander.send_velocity_world_setpoint(
                vx,
                vy,
                vz,
                yaw_rate,
            )
            with self._state_lock:
                sent_at = time.monotonic()
                if self._last_velocity_sent_at is not None:
                    self._max_velocity_gap_s = max(
                        self._max_velocity_gap_s,
                        sent_at - self._last_velocity_sent_at,
                    )
                self._last_velocity_sent_at = sent_at
                self._sent_velocity += 1

    def send_external_position(self, x, y, z):
        """Forward one external position measurement to the estimator."""
        with self._state_lock:
            started = self._started
            command = (self._command_generation, x, y, z)
        if started and self._dispatcher.submit_external_position(self, command):
            return
        self._send_external_position_now(x, y, z)

    def _send_external_position_now(self, x, y, z):
        """Send external position from the dedicated worker."""
        with self._radio_lock:
            self.scf.cf.extpos.send_extpos(x, y, z)

    def _send_external_position_if_current(self, generation, x, y, z):
        """Send a queued pose only if it predates no safety command."""
        with self._radio_lock:
            with self._state_lock:
                if generation != self._command_generation:
                    return
            self.scf.cf.extpos.send_extpos(x, y, z)
            with self._state_lock:
                self._sent_external_position += 1

    def _record_send_error(self, error):
        """Retain the newest dispatcher exception for health diagnostics."""
        with self._state_lock:
            self._last_send_error = str(error)

    def _record_stale_velocity(self):
        """Count a command that expired before its next radio frame."""
        with self._state_lock:
            self._stale_velocity_drops += 1
            self._last_velocity_sent_at = None

    def _record_dispatch_overrun(self, skipped_frames, delay_s):
        """Retain missed-frame evidence without logging in the worker."""
        with self._state_lock:
            self._dispatch_frames_skipped += skipped_frames
            self._max_dispatch_delay_s = max(
                self._max_dispatch_delay_s,
                delay_s,
            )

    def consume_send_statistics(self):
        """Return handoff counters plus cflib's measured radio statistics."""
        with self._state_lock:
            radio_statistics_age_s = None
            if self._radio_statistics_received_at is not None:
                radio_statistics_age_s = (
                    time.monotonic() - self._radio_statistics_received_at
                )
            result = {
                'velocity': self._sent_velocity,
                'external_position': self._sent_external_position,
                'stale_velocity': self._stale_velocity_drops,
                'skipped_frames': self._dispatch_frames_skipped,
                'max_dispatch_delay_s': self._max_dispatch_delay_s,
                'max_velocity_gap_s': self._max_velocity_gap_s,
                'error': self._last_send_error,
                'radio_uplink_rate_hz': self._radio_uplink_rate_hz,
                'radio_uplink_payload_fraction': (
                    self._radio_uplink_payload_fraction
                ),
                'radio_link_quality': self._radio_link_quality,
                'radio_statistics_age_s': radio_statistics_age_s,
            }
            self._sent_velocity = 0
            self._sent_external_position = 0
            self._stale_velocity_drops = 0
            self._dispatch_frames_skipped = 0
            self._max_dispatch_delay_s = 0.0
            self._max_velocity_gap_s = 0.0
            self._last_send_error = None
        return result

    def stop(self):
        """Send a motor-stop setpoint, which does not perform a landing."""
        self._clear_pending()
        with self._radio_lock:
            self.scf.cf.commander.send_stop_setpoint()

    def stop_and_disarm(self):
        """Stop setpoint streaming, stop the motors, and request disarming."""
        self._clear_pending()
        with self._radio_lock:
            self.scf.cf.commander.send_stop_setpoint()
            self.scf.cf.commander.send_notify_setpoint_stop()
            self.scf.cf.supervisor.send_arming_request(False)


class SimulatedCrazyflieInterface:
    """A command interface that forwards velocity to a ROS publisher callback."""

    def __init__(self, velocity_callback):
        """Store the callback used to publish simulated velocity commands."""
        self.velocity_callback = velocity_callback
        self.armed = False

    def arm(self):
        """Mark the simulated vehicle as armed."""
        self.armed = True
        return ('Is armed', 'Can fly')

    def disarm(self):
        """Mark the simulated vehicle as disarmed."""
        self.armed = False

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Forward one world-frame velocity command to Gazebo."""
        if self.armed:
            self.velocity_callback(vx, vy, vz, yaw_rate)

    def stop(self):
        """Publish a zero velocity command."""
        self.velocity_callback(0.0, 0.0, 0.0, 0.0)

    def stop_and_disarm(self):
        """Stop the simulated vehicle and mark it disarmed."""
        self.stop()
        self.disarm()
