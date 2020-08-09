import numpy as np
from scipy import integrate,signal
from scipy.signal import butter, lfilter
from scipy.spatial.transform import Rotation as R

from FILE_IN import FILE_READ_IN
import IMU_Graphing
import statistics 



# %% Function to rotate into vector by a quaternion. 
def Rotate(But_Accel_N,RAW_POSEST,Line_count,Time_s):
    IMU_Graphing.Plot_Raw_Split(But_Accel_N[1000:,...],Time_s[1000:],'Raw Acceleration Data')
    IMU_Graphing.Plot_Raw_Split(RAW_POSEST[1000:,...],Time_s[1000:],'Raw Pose Estimates (deg)')
    
    #Store pose estimates as a array of Rotation objects. See scipy.spatial.transform library.
    Quat_POSEST = [R.from_euler('xyz',RAW_POSEST[i,...],degrees = True) for i in range(Line_count)]
    
    Accel_N = np.zeros((Line_count,3))
    
    #Rotate Acceleration in the body frame, to the navigation frame.
    for line in range(Line_count):
        Accel_N[line] = Quat_POSEST[line].apply(But_Accel_N[line])
        
    Accel_N[:,1] = Accel_N[:,1]*0.1
    IMU_Graphing.Plot_Raw_Split(Accel_N[1000:,...],Time_s[1000:],'Rotated Acceleration Data')
    
    return Accel_N, RAW_POSEST
    
