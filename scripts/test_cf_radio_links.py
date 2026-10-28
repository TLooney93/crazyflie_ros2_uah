"""Open and close each configured Crazyflie link without sending commands."""

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cf_control.config import CACHE_DIR, DRONES


def main():
    cflib.crtp.init_drivers()
    failed = False
    for device_id in ('cf1', 'cf2'):
        uri = DRONES[device_id]
        print(f'Testing {device_id}: {uri}', flush=True)
        try:
            with SyncCrazyflie(
                    uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))):
                print(f'PASS: {device_id} connected', flush=True)
        except Exception as error:
            failed = True
            print(f'FAIL: {device_id}: {error}', flush=True)
        time.sleep(0.5)
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
