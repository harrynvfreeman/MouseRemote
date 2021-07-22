import serial
import quaternion as quat
import numpy as np
import pyautogui

pyautogui.PAUSE = 0.001
mouse_x_centre = 860
mouse_y_centre = 540

mouse_x_min = 0
mouse_y_min = 0

mouse_x_max = 2*mouse_x_centre
mouse_y_max = 2*mouse_y_centre

port = "/dev/cu.usbmodem1411"
#port = "/dev/cu.usbmodem1421"
baud = 115200
max_val = 60*256
should_stop = True

fs = 256
Ts = 1/fs

sensitivity = 30

accel_coeff = 2 / (1<<15)
accel_gravity = np.array([0., 0., 1.])
accel_error_coeff = 0.02

gyro_coeff = 250 / (1<<15)
gyro_offset = [0.031553361460664994, -0.01297136301409727, 0.00843597887174631]

x_size = 1.5
y_size = 3.
z_size = 0.01

arduino = serial.Serial(port, baud)
_ = arduino.readline()

buffer_string = ''
to_run = True
init = True
while to_run:    
    buffer_string = buffer_string + arduino.read(arduino.inWaiting()).decode()
    if not '\n' in buffer_string:
        continue
    
    lines = buffer_string.split('\n')
    buffer_string = lines[-1]
    for data in lines[0:-1]:
        data = data.split("|")
        accel_body = np.array([float(data[4]), float(data[5]), float(data[6])])*accel_coeff
        accel_body = accel_body / np.linalg.norm(accel_body)
        if (init):
            q = quat.getQuat(accel_gravity, accel_body)
            qInv = quat.quatInv(q)
            _, pitch_zero, yaw_zero = quat.quat2eul(qInv)
            mouse_x = 860
            mouse_y = 540
            init = False
        
        accel_body_approx = quat.quatTransform(q, accel_gravity)
        accel_error = -np.cross(accel_body_approx, accel_body)
        
        gyro = np.radians(np.array([float(data[1]), float(data[2]), float(data[3])])*gyro_coeff) - gyro_offset
        gyro_update = (1-accel_error_coeff)*gyro + (accel_error_coeff)*accel_error
        qInv = quat.updateQuat(qInv, gyro_update, Ts)
        q = quat.quatInv(qInv)
        
        _, pitch, yaw = quat.quat2eul(qInv)
        delta_pitch = pitch - pitch_zero
        delta_yaw = yaw - yaw_zero
        pitch_zero = pitch
        yaw_zero = yaw
        
        mouse_x = mouse_x - np.degrees(delta_yaw)*sensitivity
        mouse_y = mouse_y + np.degrees(delta_pitch)*sensitivity    
        
    pyautogui.moveTo(mouse_x, mouse_y, duration=0)
    
    if should_stop and int(data[0]) >= max_val - 1:
        to_run = False
    
print('DONE')