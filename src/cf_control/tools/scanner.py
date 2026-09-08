import cflib.crtp

cflib.crtp.init_drivers()

available = cflib.crtp.scan_interfaces()

for uri, comment in available:
    print(uri, comment)
