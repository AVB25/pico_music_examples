import numpy as np
import matplotlib.pyplot as plt
import serial 

COLON_ASCII = 58

ser = serial.Serial("COM3", baudrate=115200)

def filter_line(line):
    colon_idx = 0
    for idx, i in enumerate(line):
        if int(i) == COLON_ASCII:
            colon_idx = idx
    return int(line[colon_idx+2:-2])

# ser.write(b'c')
# print(int(filter_line(ser.readline())))
# ser.close()

samples = []
for i in range(1000):
    ser.write(b'c')
    line = ser.readline()
    samples.append(filter_line(line) % 2**16)
    # samples.append(filter_line(line))
    # print(int(line[-5:-2]))
ser.close()

plt.plot(samples)
plt.show()

