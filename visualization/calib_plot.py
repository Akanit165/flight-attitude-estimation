import matplotlib.pyplot as plt
import csv
import pandas as pd
import numpy as np

def raw():
    data = pd.read_csv('rawdata.txt',sep='\t',header=None)
    data = pd.DataFrame(data)
    x = data[0]
    y = data[1]
    z = data[2]

    # ax = plt.subplot(111, projection = '3d')
    # plt.suptitle("Magnetometer Calibration")
    # ax.scatter(x,y,z,'o')
    # plt.grid(True)
    # plt.show()
    return data

def calib():
    data = pd.read_csv('rawdata.txt',sep='\t',header=None)
    data = pd.DataFrame(data)
    # raw_x = data[0]
    # raw_y = data[1]
    # raw_z = data[2]
    bias = np.array([0.376457,0.119874,-0.165460])
    scale = np.array([[0.880161, -0.029439, -0.022056],
                      [-0.029439, 0.969466, 0.014079],
                      [-0.022056, 0.014079, 0.949999]])
    rawdata = np.array(data)
    
    for i in range(np.shape(rawdata)[0]):
        rawdata[i,:] = rawdata[i,:]-bias
    calib_data = np.matmul(rawdata,scale)
    return calib_data

def plot():
    plt.style.use("dark_background")
    # raw_x, raw_y, raw_z = raw()
    raw_x = raw()[0]
    raw_y = raw()[1]
    raw_z = raw()[2]
    cal_x = calib()[:,0]
    cal_y = calib()[:,1]
    cal_z = calib()[:,2]
    
    plt.suptitle("Magnetometer Calibration")
    
    ax1 = plt.subplot(221, projection = '3d')
    ax1.scatter(raw_x,raw_y,raw_z,'bo')
    ax1.scatter(cal_x,cal_y,cal_z,'ro')
    
    ax2 = plt.subplot(222)
    ax2.plot(raw_x,raw_y,'bo')
    ax2.plot(cal_x,cal_y,'ro')
    ax2.set_xlabel("X (gauss)")
    ax2.set_ylabel("Y (gauss)")
    ax2.grid(True)
    
    ax3 = plt.subplot(223)
    ax3.plot(raw_x,raw_z,'bo')
    ax3.plot(cal_x,cal_z,'ro')
    ax3.set_xlabel("X (gauss)")
    ax3.set_ylabel("Z (gauss)")
    ax3.grid(True)
    
    ax4 = plt.subplot(224)
    ax4.plot(raw_y,raw_z,'bo')
    ax4.plot(cal_y,cal_z,'ro')
    ax4.set_xlabel("Y (gauss)")
    ax4.set_ylabel("Z (gauss)")
    ax4.grid(True)
    
    plt.show()
    
# plot()