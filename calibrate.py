# Calibration of GY511 Magnetometer

import time
import serial
import struct
import csv
import visualization.calib_plot as plot

def receiveData():
    
    rx = serial.Serial(port = "/dev/ttyUSB0", baudrate = 115200)
    
    with open('raw_data/rawdata.txt', 'w') as f:
        writer = csv.writer(f, delimiter='\t')
    
    try:
        while True:
            header = 0xff
            bufferLength = 13
            bufferRead = rx.read(bufferLength)
            data = list(bufferRead)
            #print(data)
            if data[0] == header:
                magData = list(struct.unpack('fff',bytes(data[1:bufferLength])))
                magX, magY, magZ = magData
                print("X : ",magX, " Y : ", magY, " Z : ", magZ)
                
                with open('raw_data/rawdata.txt', 'a') as f:
                    # write a row to the csv file
                    writer = csv.writer(f, delimiter='\t')
                    writer.writerow(magData)

    except KeyboardInterrupt:
        print("Finished")
        plot.plot()

if __name__ == "__main__":
    receiveData()
        
    
    