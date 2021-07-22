# MouseRemote
Control mouse using Arduino Uno and MPU9255

To run, flash IMU.ino onto an Arduino Uno connected to an MPU9255 module.  Connect the Arduino to a serial port on the computer, and change the "port" variable in mouseRemote.py.  Also change mouse_x_centre and mouse_y_centre depending on the size of your monitor.  Then run "python3 mouseRemote.py".

File descriptions:

2DCalibrate.py - Used to find gyroscope offsets

IMU.ino - Program that should be flashed to the arduino to read gyroscope and acceleromter values at 256Hz

ReadWriteExample.ino - Read from and write to MPU9255 test register

mousePlot.py - Plots the orientation of the MPU9255

mouseRemote.py - Moves the mouse on screen based off of MPU9255 readings

pyautoguiExample.py - Example of how to use the pyautogui library

quaternion.py - Custom quaternion library to track orientation

readSerialExample.py - Example of how to use the serial library
