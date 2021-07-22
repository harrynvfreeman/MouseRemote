import serial

#/dev/tty vs /dev/cu???
port = "/dev/cu.usbmodem1411"
#port = "/dev/cu.usbmodem1421"
baud = 115200
max_val = 60*256
should_stop = True

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
    #data = lines[-2]
    buffer_string = lines[-1]
    for data in lines[0:-1]:
    	print(data)