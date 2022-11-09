# This visualization file is included the raspberry pi cpu status
import sys
import numpy as np
from math import *
sys.path.append('..')
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def animate(i):

    ins_data = pd.read_csv('log_data/ins_data.csv')
    pitch = ins_data['pitch']
    yaw = ins_data['yaw']
    roll = ins_data['roll']
    
    ax = plt.subplot(111,projection='3d')

    ax.clear()
    
    ax.set_xlabel('$x\ (LVLH_{North})$')
    ax.set_ylabel('$y\ (LVLH_{East})$')
    ax.set_zlabel('$z\ (LVLH_{Down})$')
    
    CY = cos(radians(yaw))
    SY = sin(radians(yaw))
    
    CP = cos(radians(pitch))
    SP = sin(radians(pitch))
    
    CR = cos(radians(roll))
    SR = sin(radians(roll))
    
    # Visualize in north east down (neglect axis from matplotlib)
    dcm_yaw = np.array([[ CY, -SY, 0],
                        [ SY, CY, 0],
                        [  0,  0, 1]])
    
    dcm_pitch = np.array([[ CP,  0,  SP],
                          [  0,  1,   0],
                          [ -SP,  0,  CP]])
    
    dcm_roll = np.array([[ 1,   0,  0], 
                         [ 0,  CR, SR],
                         [ 0,  -SR, CR]])
    
    dcm = np.matmul(dcm_yaw,np.matmul(dcm_pitch,dcm_roll))

    # Star tracker body frame
    ub = [dcm[0,0],dcm[0,1],dcm[0,2]]
    vb = [dcm[1,0],dcm[1,1],dcm[1,2]]
    wb = [dcm[2,0],dcm[2,1],dcm[2,2]]

    ax.set_xlim(-1,1,0.5)
    ax.set_ylim(-1,1,0.5)
    ax.set_zlim(-1,1,0.5)
    ax.set_xticks(np.arange(-1, 1.2,0.5))
    ax.set_yticks(np.arange(-1, 1.2,0.5))
    ax.set_zticks(np.arange(-1, 1.2,0.5))

    
    start = [0,0,0]
    ax.quiver(start[0],start[1],start[2], ub[0],ub[1],ub[2], colors = 'w')
    ax.quiver(start[0],start[1],start[2], vb[0],-vb[1],-vb[2], colors = 'yellow')
    ax.quiver(start[0],start[1],start[2], wb[0],-wb[1],-wb[2], colors = 'yellow')
    
    ax.view_init(15,15)
    plt.tight_layout()
 
def perform_plot():
    plt.style.use('dark_background')
    fig = plt.figure()
    fig.suptitle("Attitude Testing Monitor (Extened Kalman Filter)")
    ani = FuncAnimation(plt.gcf(), animate, interval = 10)
    plt.show()

perform_plot()

