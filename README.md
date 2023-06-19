# Embedded-MAVLink
Code for establishing mavlink communication between Pixhawk Cube Orange and two development boards a) Teensy 4.0/4,1 b) STM32 f446ZE. The code is used to extract attitude information from pixhawk IMU.  MAVLink 2 official library has been used for both boards (https://github.com/mavlink/c_library_v2). 

Place the contents of c_library_v2 in a separate directory in your project workspace before execution (I named mine src).
