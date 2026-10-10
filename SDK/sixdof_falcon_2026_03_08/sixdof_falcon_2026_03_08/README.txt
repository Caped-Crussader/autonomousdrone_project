Copyright (c) 2017-2026 Sixdof Space All rights reserved. 
Proprietary and Confidential

This folder contains the Sixdof Falcon package: Falcon Studio and the SdsFalcon SDK.

To get started, review the "Sixdof Falcon User Guide" from the docs directory.

The contents include:

+---bin						- Shared libraries for supported platforms and Falcon Studio executables for Windows and Linux
|   +---jetson
|   |       libSdsFalcon.so
|   |
|   +---rpi32
|   |       libSdsFalcon.so
|   |
|   +---rpi64
|   |       libSdsFalcon.so
|   |
|   +---ubuntu
|   |       libSdsFalcon.so
|   |       sixdof_falcon_studio
|   |
|   +---windows_gcc
|   |       SdsFalcon.dll
|   |       sixdof_falcon_studio.exe
|   |
|   \---windows_msvc
|           SdsFalcon.dll
|           SdsFalcon.lib
|
+---doc						- Documentation for Falcon Studio and the SdsFalcon SDK
|       SdsFalcon API.pdf
|       Sixdof Falcon User Guide.pdf
|       Python Bindings.pdf
|       Build Examples.pdf
|       Sixdof Software License.pdf
|
+---examples					- C++ build examples
|       CMakeLists.txt
|       field_of_view_report.cpp
|       gyro_calibration.cpp
|       heartbeat.cpp
|       heartbeat_manager.cpp
|       playback.cpp
|       relative_angle_tracking.cpp
|       sixdof_tracking.cpp
|       dev_features.cpp
|
+---include					- Header files
|       SdsFalcon.h
|       SdsShutterControl.h
|	DevFeatures.h
|
\---python_bindings				- Python bindings
        CMakeLists.txt
        pyFalcon.cpp
	pyShutterControl.cpp
	falcon_shutter_control.py
        falcon_test_bindings.py
        falcon_test_bindings_record.py
        falcon_test_bindings_playback.py


Version Information:

| Program                    | Version       | Release Date  |
|----------------------------|---------------|---------------|
| SdsFalcon SDK              | 2.0.1         | 2026-03-08    |
| Sixdof Falcon Studio       | 2.0.0         | 2026-01-08    |
| Sixdof Falcon User Guide   | 6             | 2026-01-08    |
| Python Bindings            | 5             | 2026-01-08    |
| Build Examples             | 6             | 2026-01-08    |



