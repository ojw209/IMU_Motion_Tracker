'''
Auther: Oliver West

Ref frame tutorial.

https://arxiv.org/pdf/1704.06053.pdf
http://work.thaslwanter.at/skinematics/html/

Note: World frame 

Let b be the body frame, n be the navigation frame and e be the world frame.
'''

# $easy_install numpy scipy Sphinx numpydoc nose pykalman
import os
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz
from scipy.spatial.transform import Rotation as R
from scipy import signal


#Import local packages.
from Data_In import File_read
from Quat_Func import eul_to_quat,quat_to_eul,quat_conj,quat_prod,axisangle_to_q,normalize
from IMU_Plotting import Angle_Plots,Position_Plots



#Low Pass Filter
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



#Paramters to twiddle with.
Sample_Freq = 100
Int_position = [0,0,0]


#Set working directory.
wd= os.path.dirname(os.path.dirname(os.path.abspath("__file__")))
os.chdir(wd)

#Set path and directory. [Need's to be tested for portability.]
FILE_Path = 'Raw_Data/IMU_3.CSV'

#Read in data.
RolPitYaw_b, RolPitYaw_dt_b, Accel_b, Magnet_b,line_count = File_read(FILE_Path)


#Define time vector [Needs correcting.]
t = np.array((range(line_count)))
#Change angles to quartion sytem.
QRolPitYaw_b = eul_to_quat(RolPitYaw_b)
QMagnet_b = eul_to_quat(Magnet_b)
#[Note: need to double check for any hidden complexities in coordinate changes with derivatives]
QRolPitYaw_b_dt = eul_to_quat(RolPitYaw_dt_b)


#Conversion to the navigation frame. [Note: Needs, cleaning!]
RolPitYaw_n = np.zeros((line_count,3))
Accel_n = np.zeros((line_count,3))
Accel_n_f = np.zeros((line_count,3))
Vel_n = np.zeros((line_count,3))
Vel_n_f = np.zeros((line_count,3))
Pos_n = np.zeros((line_count,3))
Pos_n_f = np.zeros((line_count,3))

for i in range(line_count):
    #Determine our z location, via intial calibration. [Note: Build in failsafe, if forces being exerted during calibration]
    Calib_Z_Vector_Mag = math.sqrt(Accel_b[i,0]**2 + Accel_b[i,1]**2 + Accel_b[i,2]**2)
    Calib_Z_Vector_Angle = np.degrees(np.array([math.cos(Accel_b[i,0]/Calib_Z_Vector_Mag ),math.cos(Accel_b[i,1]/Calib_Z_Vector_Mag ),math.cos(Accel_b[i,2]/Calib_Z_Vector_Mag )]))
    Calib_Z_Vector = -Accel_b[i,:]/Calib_Z_Vector_Mag 
    
    #Determine North,[East = Down x Magnetic_resultant -> North = East x Down]
    Calib_Magnet_Mag = math.sqrt(Magnet_b[i,0]**2 + Magnet_b[i,1]**2 + Magnet_b[i,2]**2)
    Calib_Magnet_Vector = Magnet_b[i,:]/Calib_Magnet_Mag
    
    #Calculate reference frame.
    East_vector = normalize(np.cross(Calib_Z_Vector,Calib_Magnet_Vector))
    North_vector = normalize(np.cross(East_vector, Calib_Z_Vector))

    NB_R_Frame = R.from_euler('xyz',np.array([North_vector,East_vector,Calib_Z_Vector]), degrees=True)
    
    r1, r2, r3 = NB_R_Frame[0,...], NB_R_Frame[1,...], NB_R_Frame[2,...]
    r3 = r2*r1
    
    RolPitYaw_n[i,...] = r3.apply(RolPitYaw_b[i,...])
    Accel_n[i,...] = r3.apply(Accel_b[i,...]) - np.array([0,0,9.81])
    

##Butter Worth Filter
# Filter requirements.
order = 1
fs = 30      # sample rate, Hz
cutoff = .3
#Low Pass FIlter 
b, a = butter_lowpass(cutoff, fs, order)
for i in range(3):
    Accel_n_f[...,i] = butter_lowpass_filter(Accel_n[...,i], cutoff, fs, order)


for i in range(line_count):
    for j in range(3):
        Vel_n[i,j] = np.trapz(Accel_n[0:i,j],dx = 1/30)
        Vel_n_f[i,j] = np.trapz(Accel_n_f[0:i,j],dx = 1/30)
Vel_n[...,0] = signal.detrend(Vel_n[...,0]) 
Vel_n[...,1] = signal.detrend(Vel_n[...,1])  
Vel_n[...,2] = signal.detrend(Vel_n[...,2])   
  
Vel_n_f[...,0] = signal.detrend(Vel_n_f[...,0]) 
Vel_n_f[...,1] = signal.detrend(Vel_n_f[...,1])  
Vel_n_f[...,2] = signal.detrend(Vel_n_f[...,2])     



for i in range(line_count):
    for j in range(3):
        Pos_n[i,j] = np.trapz(Vel_n[0:i,j],dx = 1/30)  
        Pos_n_f[i,j] = np.trapz(Vel_n_f[0:i,j],dx = 1/30)





Angle_Plots(RolPitYaw_n,t)
Position_Plots(Accel_n,Accel_n_f,Vel_n,Vel_n_f,Pos_n,Pos_n_f,t )