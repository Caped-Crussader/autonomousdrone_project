import sys
import os
import time
import numpy as np
sys.path.append(os.getcwd())
try:
    import pySdsSdsFalcon as sdk  # Import all of the pybind11 cpp functions
    import pySdsShutterControl as sc
except ImportError:
    print("Failed to import pySdsFalcon.pyd (or pySdsFalcon.so on Linux). Make sure that SdsFalcon.dll and pySdsFalcon.pyd are in the same folder as this file")

class MyShutterControl(sc.IShutterControl):
    def __init__(self):
        super().__init__()
        self.t0 = time.time()

    def updateShutter(self, currentShutter, peaks):
        t = time.time()
        if t - self.t0 >= 0.25:
            self.setShutterFixed((currentShutter + 1) % 16)
            self.t0 = t
       

version = sdk.getVersion()
print(version.getString())

falconManager = sdk.createFalconManager()

falconManager.registerMessageCallback(lambda message: print(message.message))

def relative_angle_callback(col):
    for ra in col:
        print(f'id: {ra.id} ang_x: {ra.x_angle * 180.0 / np.pi:.2f} ang_z: {ra.z_angle * 180.0 / np.pi:.2f}')

#falconManager.registerRelativeAngleCallback(lambda col: relative_angle_callback(col))

falconManager.registerCurrentShutterCallback(lambda cs: print(cs.shutter_value))

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

print("Default Shutter")
time.sleep(15)

print("Python Shutter")
shutter_control = MyShutterControl()
falconManager.setShutterControl(shutter_control)
time.sleep(15)

print("Default Shutter")
falconManager.loadDefaultShutterControl()
time.sleep(15)

falconManager.stop()
