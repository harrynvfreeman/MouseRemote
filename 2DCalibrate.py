import numpy as np

#Used to calibrate 2D mouse
#Might be applicable to 3D
#but most likely not since gravity offsets
#would change (for accel only, gyro fine)

#get data
data_path = "/Users/HarryFreeman/Documents/Arduino/IMU/data/stationary2.txt"
data = np.genfromtxt(data_path, delimiter='|')

#sacale accel data
accel_coeff = 2 / (1<<15)
data[:, 4:7] = data[:, 4:7]*accel_coeff

#scale gryo data
gyro_coeff = 250 / (1<<15)
data[:, 1:4] = np.radians(data[:, 1:4]*gyro_coeff)

#get data
accel_x = data[:,4]
accel_y = data[:,5]
accel_z = data[:,6]

gyro_x = data[:,1]
gyro_y = data[:,2]
gyro_z = data[:,3]

accel_x_offset = np.mean(accel_x)
accel_y_offset = np.mean(accel_y)
accel_z_offset = np.mean(accel_z)

gyro_x_offset = np.mean(gyro_x)
gyro_y_offset = np.mean(gyro_y)
gyro_z_offset = np.mean(gyro_z)

print('accel x offset: ' + str(accel_x_offset))
print('accel y offset: ' + str(accel_y_offset))
print('accel z offset: ' + str(accel_z_offset))

print('gyro x offset: ' + str(gyro_x_offset))
print('gryo y offset: ' + str(gyro_y_offset))
print('gyro z offset: ' + str(gyro_z_offset))