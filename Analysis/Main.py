'''
Auther: Oliver West

Ref frame tutorial.

https://arxiv.org/pdf/1704.06053.pdf

Note: World frame 

Let b be the body frame, n be the navigation frame and e be the world frame.
'''

# $easy_install numpy scipy Sphinx numpydoc nose pykalman
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
from scipy.spatial.transform import Rotation as R

#Import local packages.
from Data_In import File_read
from Quat_Func import eul_to_quat,quat_to_eul,quat_conj,quat_prod,axisangle_to_q,normalize


#Paramters to twiddle with.
Sample_Freq = 100
Int_position = [0,0,0]


#Set working directory.
wd= os.path.dirname(os.path.dirname(os.path.abspath("__file__")))
os.chdir(wd)

#Set path and directory. [Need's to be tested for portability.]
FILE_Path = 'Raw_Data/IMU_2.CSV'

#Read in data.
RolPitYaw_b, RolPitYaw_dt_b, Accel_b, Magnet_b,line_count = File_read(FILE_Path)

#Change angles to quartion sytem.
QRolPitYaw_b = eul_to_quat(RolPitYaw_b)
QMagnet_b = eul_to_quat(Magnet_b)
#[Note: need to double check for any hidden complexities in coordinate changes with derivatives]
QRolPitYaw_b_dt = eul_to_quat(RolPitYaw_dt_b)

RolPitYaw_n = np.zeros((line_count,3))
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