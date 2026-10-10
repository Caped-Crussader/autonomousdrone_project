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

def pose_6dof_callback(pose):
    print(f'X: {pose.x:.2f} Y: {pose.y:.2f} Z: {pose.z:.2f} Qw: {pose.qw:.2f} Qx: {pose.qx:.2f}  Qy: {pose.qy:.2f}  Qz: {pose.qz:.2f} ')

falconManager.registerPose6DofCallback(lambda col: pose_6dof_callback(col))

net_config = sdk.NetworkConfig()
net_config.sensor_id = 246
net_config.sensor_network_name = 'Ethernet 2'

config = sdk.Config()
config.networkConfig = net_config

b0 = sdk.Beacon()
b0.id = 753
b0.x = 0.000
b0.y = 0.0
b0.z = 0.000

b1 = sdk.Beacon()
b1.id = 752
b1.x = 0.174
b1.y = 0.0
b1.z = 0.145

b2 = sdk.Beacon()
b2.id = 765
b2.x = 0.162
b2.y = 0.0
b2.z = -0.118

b3 = sdk.Beacon()
b3.id = 760
b3.x = -0.163
b3.y = 0.0
b3.z = -0.120

b4 = sdk.Beacon()
b4.id = 759
b4.x = -0.179
b4.y = 0.0
b4.z = 0.145

config.map = [b0, b1, b2, b3, b4]

playbackSettings = sdk.DataPlaybackSettings()
playbackSettings.mode = sdk.DataPlaybackMode.PLAYBACK
playbackSettings.path = "C:/ReleasesFolder/python_bindings_test/recording.bin"

init_success = falconManager.initialize(config, playbackSettings)
if not init_success:
    print('Initialization failed')
else:
    print('Initialization success')

falconManager.start()

time.sleep(10)

falconManager.stop()
