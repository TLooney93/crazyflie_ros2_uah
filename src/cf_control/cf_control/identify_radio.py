"""Blink one Crazyflie's LEDs to identify a configured radio address safely."""

import argparse
import time

from cf_control.config import CACHE_DIR, DRONES
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp


def _parse_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            'Connect to one Crazyflie and blink its onboard LEDs without '
            'arming or commanding its motors.'
        )
    )
    parser.add_argument(
        'drone_or_uri',
        help='Configured drone name such as cf1, or a complete radio URI',
    )
    parser.add_argument(
        '--seconds',
        type=float,
        default=8.0,
        help='Identification duration (default: 8 seconds)',
    )
    return parser.parse_args(args)


def _resolve_uri(drone_or_uri):
    if drone_or_uri.startswith('radio://'):
        return drone_or_uri
    try:
        return DRONES[drone_or_uri]
    except KeyError as error:
        choices = ', '.join(sorted(DRONES))
        raise ValueError(
            f'Unknown drone {drone_or_uri!r}; configured names: {choices}'
        ) from error


def blink_identity(uri, duration_s):
    """Blink all LEDs on ``uri`` for a bounded duration without arming."""
    if duration_s <= 0.0 or duration_s > 60.0:
        raise ValueError('--seconds must be greater than 0 and at most 60')

    cflib.crtp.init_drivers()
    print(f'Connecting to {uri}')
    print('No arming or motor command will be sent.')
    with SyncCrazyflie(
        uri,
        cf=Crazyflie(rw_cache=str(CACHE_DIR)),
    ) as scf:
        deadline = time.monotonic() + duration_s
        leds_on = False
        try:
            while time.monotonic() < deadline:
                leds_on = not leds_on
                scf.cf.param.set_value(
                    'led.bitmask',
                    255 if leds_on else 0,
                )
                time.sleep(0.35)
        finally:
            scf.cf.param.set_value('led.bitmask', 0)
            time.sleep(0.1)
    print('Identification complete; LEDs restored to firmware control.')


def main(args=None):
    parsed = _parse_args(args)
    try:
        uri = _resolve_uri(parsed.drone_or_uri)
        blink_identity(uri, parsed.seconds)
    except (ValueError, RuntimeError) as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
