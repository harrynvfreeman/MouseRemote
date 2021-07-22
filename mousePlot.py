import serial
import matplotlib.pyplot as plt
import quaternion as quat
import numpy as np

port = "/dev/cu.usbmodem1411"
#port = "/dev/cu.usbmodem1421"

baud = 115200
max_val = 60*256
should_stop = True

plot_axis = False

fs = 256
Ts = 1/fs

sensitivity = 30

accel_coeff = 2 / (1<<15)
accel_gravity = np.array([0., 0., 1.])
accel_error_coeff = 0.02

gyro_coeff = 250 / (1<<15)
gyro_offset = [0.031553361460664994, -0.01297136301409727, 0.00843597887174631]

x_axis_body = np.array([1., 0., 0.])
y_axis_body = np.array([0., 1., 0.])
z_axis_body = np.array([0., 0., 1.])

x_size = 1.5
y_size = 3.
z_size = 0.01

vertex_0 = np.array([-x_size/2, -y_size/2, -z_size/2])
vertex_1 = np.array([-x_size/2, -y_size/2, z_size/2])
vertex_2 = np.array([-x_size/2, y_size/2, -z_size/2])
vertex_3 = np.array([-x_size/2, y_size/2, z_size/2])
vertex_4 = np.array([x_size/2, -y_size/2, -z_size/2])
vertex_5 = np.array([x_size/2, -y_size/2, z_size/2])
vertex_6 = np.array([x_size/2, y_size/2, -z_size/2])
vertex_7 = np.array([x_size/2, y_size/2, z_size/2])

plt.ion()
fig=plt.figure()
ax = plt.axes(projection='3d')
ax.view_init(16, 177)

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
            init = False
        
        accel_body_approx = quat.quatTransform(q, accel_gravity)
        accel_error = -np.cross(accel_body_approx, accel_body)
        
        gyro = np.radians(np.array([float(data[1]), float(data[2]), float(data[3])])*gyro_coeff) - gyro_offset
        gyro_update = (1-accel_error_coeff)*gyro + (accel_error_coeff)*accel_error
        qInv = quat.updateQuat(qInv, gyro_update, Ts)
        q = quat.quatInv(qInv)
    
    vertex_0_inertial = quat.quatTransform(qInv, vertex_0)
    vertex_1_inertial = quat.quatTransform(qInv, vertex_1)
    vertex_2_inertial = quat.quatTransform(qInv, vertex_2)
    vertex_3_inertial = quat.quatTransform(qInv, vertex_3)
    vertex_4_inertial = quat.quatTransform(qInv, vertex_4)
    vertex_5_inertial = quat.quatTransform(qInv, vertex_5)
    vertex_6_inertial = quat.quatTransform(qInv, vertex_6)
    vertex_7_inertial = quat.quatTransform(qInv, vertex_7)
    
    plt.cla()
    
    _=ax.set_xlim3d(-4, 4)
    _=ax.set_ylim3d(-4, 4)
    _=ax.set_zlim3d(-4, 4)
    
    _=ax.plot([vertex_0_inertial[0], vertex_1_inertial[0]], [vertex_0_inertial[1], vertex_1_inertial[1]], [vertex_0_inertial[2], vertex_1_inertial[2]], c='blue')
    _=ax.plot([vertex_0_inertial[0], vertex_2_inertial[0]], [vertex_0_inertial[1], vertex_2_inertial[1]], [vertex_0_inertial[2], vertex_2_inertial[2]], c='blue')
    _=ax.plot([vertex_0_inertial[0], vertex_4_inertial[0]], [vertex_0_inertial[1], vertex_4_inertial[1]], [vertex_0_inertial[2], vertex_4_inertial[2]], c='blue')
    
    _=ax.plot([vertex_1_inertial[0], vertex_3_inertial[0]], [vertex_1_inertial[1], vertex_3_inertial[1]], [vertex_1_inertial[2], vertex_3_inertial[2]], c='blue')
    _=ax.plot([vertex_1_inertial[0], vertex_5_inertial[0]], [vertex_1_inertial[1], vertex_5_inertial[1]], [vertex_1_inertial[2], vertex_5_inertial[2]], c='blue')

    _=ax.plot([vertex_2_inertial[0], vertex_3_inertial[0]], [vertex_2_inertial[1], vertex_3_inertial[1]], [vertex_2_inertial[2], vertex_3_inertial[2]], c='blue')
    _=ax.plot([vertex_2_inertial[0], vertex_6_inertial[0]], [vertex_2_inertial[1], vertex_6_inertial[1]], [vertex_2_inertial[2], vertex_6_inertial[2]], c='blue')
    
    _=ax.plot([vertex_3_inertial[0], vertex_7_inertial[0]], [vertex_3_inertial[1], vertex_7_inertial[1]], [vertex_3_inertial[2], vertex_7_inertial[2]], c='blue')
    
    _=ax.plot([vertex_4_inertial[0], vertex_5_inertial[0]], [vertex_4_inertial[1], vertex_5_inertial[1]], [vertex_4_inertial[2], vertex_5_inertial[2]], c='blue')
    _=ax.plot([vertex_4_inertial[0], vertex_6_inertial[0]], [vertex_4_inertial[1], vertex_6_inertial[1]], [vertex_4_inertial[2], vertex_6_inertial[2]], c='blue')
    
    _=ax.plot([vertex_5_inertial[0], vertex_7_inertial[0]], [vertex_5_inertial[1], vertex_7_inertial[1]], [vertex_5_inertial[2], vertex_7_inertial[2]], c='blue')
    
    _=ax.plot([vertex_6_inertial[0], vertex_7_inertial[0]], [vertex_6_inertial[1], vertex_7_inertial[1]], [vertex_6_inertial[2], vertex_7_inertial[2]], c='blue')
    
    if plot_axis:
    	x_axis_inertial = quat.quatTransform(qInv, x_axis_body)
    	y_axis_inertial = quat.quatTransform(qInv, y_axis_body)
    	z_axis_inertial = quat.quatTransform(qInv, z_axis_body)
    	_=ax.plot([x_axis_inertial[0], 0], [x_axis_inertial[1], 0], [x_axis_inertial[2], 0], c='cyan')
    	_=ax.plot([y_axis_inertial[0], 0], [y_axis_inertial[1], 0], [y_axis_inertial[2], 0], c='red')
    	_=ax.plot([z_axis_inertial[0], 0], [z_axis_inertial[1], 0], [z_axis_inertial[2], 0], c='green')
        
    plt.show()
    plt.pause(0.00001)
                
    if should_stop and int(data[0]) >= max_val - 1:
        to_run = False
    

print('DONE')