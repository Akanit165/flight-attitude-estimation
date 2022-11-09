import serial
import struct
from math import *
import pandas as pd
from _thread import *
import os
from log_data import log_data as log

def receiveData():
    # Create object to receive data from Arduino
    receiver = serial.Serial(port = "/dev/ttyUSB0", baudrate = 115200)
    try:
        i = 1
        start_new_thread(visualize,()) # Plot the attitude
        while True:

            header = 0xff
            dataLength = 0x0c
            bufferSize = 14
            data = receiver.read(bufferSize)
            #print(data)
            bufferAck = list(data)
            #print(bufferAck)

            # Check data
            if ((bufferAck[0] == header) and (bufferAck[1] == dataLength)):
               
                eulerRead = list(struct.unpack('fff',bytes(bufferAck[2:])))
                yaw, pitch, roll = eulerRead
                attitude = [pitch, yaw, roll]
                # log_attitude.create_log()
                print(attitude)
                #log_attitude.add_data(i, attitude = attitude)
                log_attitude.create_ins_log(attitude)
                i += 1
                
    except KeyboardInterrupt:
        print("=============Finished=============")
    
def visualize():
    os.system("python visualization/attitude_plot.py")

def main():
    print("Activate attitude monitoring system")
    log_attitude.create_log()
    receiveData()
    
if __name__ == "__main__":
    fieldname, ins_fieldname = log.create_fieldnames()
    log_attitude = log.Logdata(fieldname, ins_fieldname)
    main()
