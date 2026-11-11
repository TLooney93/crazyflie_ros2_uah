"""Tests for compact, recoverable Crazyflie telemetry."""

import time
from unittest.mock import Mock, patch

from cf_control.telemetry import (
    FLIGHT_TELEMETRY_VARIABLES,
    TelemetryReader,
)


def test_flight_profile_contains_only_required_health_values():
    """The flight block must not spend radio bytes on diagnostics."""
    assert [variable.name for variable in FLIGHT_TELEMETRY_VARIABLES] == [
        'stateEstimate.x',
        'stateEstimate.y',
        'stateEstimate.z',
        'pm.vbat',
        'stabilizer.thrust',
    ]
    assert all(
        variable.fetch_as == 'FP16'
        for variable in FLIGHT_TELEMETRY_VARIABLES
    )


def test_compact_profile_creates_one_log_block():
    """All flight health values must fit in one compact firmware block."""
    scf = Mock()
    with patch('cf_control.telemetry.LogConfig') as log_config_type:
        log_config = log_config_type.return_value
        reader = TelemetryReader(scf, callback=Mock())

        assert reader._create_log_config() is log_config

    assert log_config.add_variable.call_count == 5
    log_config.add_variable.assert_any_call('stateEstimate.x', 'FP16')
    log_config.add_variable.assert_any_call('pm.vbat', 'FP16')
    log_config.add_variable.assert_any_call('stabilizer.thrust', 'FP16')


def test_optional_thrust_is_omitted_for_older_firmware():
    """Missing diagnostic thrust must not disable essential telemetry."""
    scf = Mock()
    scf.cf.log.toc.get_element_by_complete_name.side_effect = (
        lambda name: None if name == 'stabilizer.thrust' else Mock()
    )
    with patch('cf_control.telemetry.LogConfig') as log_config_type:
        log_config = log_config_type.return_value
        reader = TelemetryReader(scf, callback=Mock())

        assert reader._create_log_config() is log_config

    assert log_config.add_variable.call_count == 4


def test_missing_required_value_fails_before_starting_stream():
    """A malformed health block must fail explicitly instead of running."""
    scf = Mock()
    scf.cf.log.toc.get_element_by_complete_name.side_effect = (
        lambda name: None if name == 'pm.vbat' else Mock()
    )
    with patch('cf_control.telemetry.LogConfig'):
        reader = TelemetryReader(scf, callback=Mock())

        try:
            reader._create_log_config()
        except RuntimeError as error:
            assert 'pm.vbat' in str(error)
        else:
            raise AssertionError('missing required telemetry was accepted')


def test_receive_callback_replaces_stale_sample_without_blocking():
    """A slow consumer must see the newest value instead of a backlog."""
    reader = TelemetryReader(Mock(), callback=Mock())
    reader.running = True

    reader._on_log_data(1, {'pm.vbat': 4.1}, None)
    reader._on_log_data(2, {'pm.vbat': 3.9}, None)

    _, data = reader._samples.get_nowait()
    statistics = reader.consume_statistics()
    assert data == {'pm.vbat': 3.9}
    assert statistics['samples'] == 2
    assert statistics['dropped'] == 1


def test_silent_telemetry_restarts_existing_log_block_with_stop_start():
    """Recovery must reuse one block and avoid delete/recreate races."""
    scf = Mock()
    log_config = Mock()
    reader = TelemetryReader(
        scf,
        callback=Mock(),
        period_ms=10,
        restart_timeout_s=0.04,
        restart_backoff_max_s=0.08,
    )
    reader._create_log_config = Mock(return_value=log_config)

    reader.start()
    time.sleep(0.11)
    reader.stop()

    scf.cf.log.add_config.assert_called_once_with(log_config)
    assert log_config.start.call_count >= 2
    assert log_config.stop.call_count >= 2
    log_config.delete.assert_called_once_with()
    assert reader.consume_statistics()['restarts'] >= 1


def test_restart_failures_do_not_kill_reader_worker():
    """A failed recovery command must leave future retries possible."""
    reader = TelemetryReader(Mock(), callback=Mock())
    log_config = Mock()
    log_config.start.side_effect = RuntimeError('radio busy')

    reader._restart_log_block(log_config)

    statistics = reader.consume_statistics()
    assert statistics['restarts'] == 1
    assert statistics['error'] == 'restart: radio busy'
