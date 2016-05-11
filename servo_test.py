import serial
import struct
import time
port = '/dev/ttyUSB0'
print("opening port " + port)
serial_port=serial.Serial()
serial_port.port=port
serial_port.timeout=2
serial_port.baudrate=115200
serial_port.open()
time.sleep(2)

def send_pid(p,i,d):
    bin = struct.pack('<HHH',p * 1000, i * 1000, d * 1000)
    serial_port.write(bin)

def send(pos):
    bin = struct.pack('<H',pos)
    serial_port.write(bin)


times = []
poss = []
refs = []

import sys
kp = float(sys.argv[1])
ki = float(sys.argv[2])
kd = float(sys.argv[3])

move_to = int(sys.argv[4])
samples = 200

send_pid(kp, ki, kd)

for i in range(samples):
    bin = serial_port.read(6)
    time, curpos, posref = struct.unpack('<HHH', bin)
    if i == samples / 4:
        send(move_to)
    poss.append(curpos)
    times.append(time)
    refs.append(posref)

print(poss)

import matplotlib.pyplot as plt
plt.plot(times, poss )
plt.plot(times, refs )
plt.xlabel('time ms')
plt.ylabel('pulses')
plt.grid(True)
plt.title('PID %f,%f,%f @ 100Hz' % (kp, ki, kd))
plt.ylim(0, move_to * 1.25)
plt.show()
