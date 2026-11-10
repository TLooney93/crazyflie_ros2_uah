"""Read hardware status from configured Crazyflies without arming them."""

import argparse

from cf_control.config import CACHE_DIR, DRONES, PREFLIGHT_MIN_BATTERY_VOLTAGE

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


BLOCKING_SUPERVISOR_STATES = {'Is crashed', 'Is locked', 'Is tumbled'}


def _parse_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            'Connect to configured Crazyflies and read battery and firmware '
            'supervisor status. This command never arms or commands motors.'
        )
    )
    parser.add_argument(
        'drone_ids',
        nargs='*',
        help='Configured names to check (default: every configured drone)',
    )
    return parser.parse_args(args)


def _selected_drones(drone_ids):
    selected = list(drone_ids) if drone_ids else list(DRONES)
    unknown = [drone_id for drone_id in selected if drone_id not in DRONES]
    if unknown:
        raise ValueError(
            f'Unknown drone(s): {", ".join(unknown)}; configured names: '
            f'{", ".join(DRONES)}'
        )
    if len(set(selected)) != len(selected):
        raise ValueError('drone names must be unique')
    return selected


def _assessment(voltage, supervisor_states):
    blockers = BLOCKING_SUPERVISOR_STATES.intersection(supervisor_states)
    reasons = []
    if voltage < PREFLIGHT_MIN_BATTERY_VOLTAGE:
        reasons.append(
            f'battery below {PREFLIGHT_MIN_BATTERY_VOLTAGE:.2f} V'
        )
    if blockers:
        reasons.append('supervisor: ' + ', '.join(sorted(blockers)))
    if 'Is armed' in supervisor_states:
        reasons.append('unexpectedly armed')
    return 'PASS' if not reasons else 'FAIL: ' + '; '.join(reasons)


def check_drone(drone_id, uri):
    """Connect, take one battery sample, and read supervisor state."""
    log_config = LogConfig(name='Preflight', period_in_ms=100)
    log_config.add_variable('pm.vbat', 'FP16')

    with SyncCrazyflie(
        uri,
        cf=Crazyflie(rw_cache=str(CACHE_DIR)),
    ) as scf:
        with SyncLogger(scf, log_config) as logger:
            _, data, _ = next(logger)
        voltage = float(data['pm.vbat'])
        supervisor_states = tuple(scf.cf.supervisor.read_state_list())

    return voltage, supervisor_states


def run_preflight(drone_ids):
    """Check selected drones sequentially and return a process exit code."""
    selected = _selected_drones(drone_ids)
    cflib.crtp.init_drivers()
    failures = 0
    print('READ-ONLY PREFLIGHT: no arming or motor commands will be sent.')
    for drone_id in selected:
        uri = DRONES[drone_id]
        try:
            voltage, states = check_drone(drone_id, uri)
            result = _assessment(voltage, set(states))
            state_text = ', '.join(states) or 'none reported'
            print(
                f'{drone_id}: {result} | {voltage:.2f} V | '
                f'{uri} | {state_text}'
            )
            failures += result != 'PASS'
        except Exception as error:
            failures += 1
            print(
                f'{drone_id}: FAIL: connection/telemetry error: '
                f'{error} | {uri}'
            )

    print(f'Summary: {len(selected) - failures}/{len(selected)} passed')
    return 0 if failures == 0 else 1


def main(args=None):
    """Run the command-line preflight check."""
    parsed = _parse_args(args)
    try:
        raise SystemExit(run_preflight(parsed.drone_ids))
    except ValueError as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
