import cflib.crtp

from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.swarm import CachedCfFactory


uris = {
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E7E6',
    'radio://0/80/2M/E7E7E7E7E5',
    'radio://0/80/2M/E7E7E7E7E4',

}


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')

    print("Opening swarm...")

    with Swarm(uris, factory=factory) as swarm:

        print("BOTH CRAZYFLIES CONNECTED")

        print("Connections closed"),


cflib.crtp.init_drivers()

for uri in uris:
    print(f"\nTrying {uri}")

    try:
        with SyncCrazyflie(uri) as scf:
            print(f"CONNECTED: {uri}")

    except Exception as e:
        print(f"FAILED: {uri}")
        print(f"Reason: {e}")
