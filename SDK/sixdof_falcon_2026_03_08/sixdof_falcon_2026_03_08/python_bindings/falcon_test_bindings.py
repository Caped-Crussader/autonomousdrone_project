import sys
import os
import time
import numpy as np
sys.path.append(os.getcwd())
try:
    import pySdsFalcon as sdk  # Import all of the pybind11 cpp functions
except ImportError:
    print("Failed to import pySdsFalcon.pyd (or pySdsFalcon.so on Linux). Make sure that SdsFalcon.dll and pySdsFalcon.pyd are in the same folder as this file")

version = sdk.getVersion()
print(version.getString())

falconManager = sdk.createFalconManager()

falconManager.registerMessageCallback(lambda message: print(message.message))

def relative_angle_callback(col):
    for ra in col:
        print(f'id: {ra.id} ang_x: {ra.x_angle * 180.0 / np.pi:.2f} ang_z: {ra.z_angle * 180.0 / np.pi:.2f}')

falconManager.registerRelativeAngleCallback(lambda col: relative_angle_callback(col))

net_config = sdk.NetworkConfig()
net_config.sensor_id = 246
net_config.sensor_network_name = 'Ethernet 2'

config = sdk.Config()
config.networkConfig = net_config

init_success = falconManager.initialize(config)
if not init_success:
    print('Initialization failed')
else:
    print('Initialization success')

falconManager.start()

time.sleep(10)

falconManager.stop()
