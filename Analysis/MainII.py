'''
Auther: Oliver West

Ref frame tutorial.

https://arxiv.org/pdf/1704.06053.pdf
http://work.thaslwanter.at/skinematics/html/

Note: World frame 

Let b be the body frame, n be the navigation frame and e be the world frame.
'''
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


# $easy_install numpy scipy Sphinx numpydoc nose pykalman
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.signal import butter, lfilter, freqz
from scipy.spatial.transform import Rotation as R



#Import local packages.
from Data_In import File_read
from Quat_Func import eul_to_quat,quat_to_eul,quat_conj,quat_prod,quat_Rot_By_Angle,ROT_FRAME_CHANGE

plt.close('all')

#Enter Unit
#Paramters to twiddle with.
Sample_Freq = 20
Int_position = [0,0,0]
unit = 0.001

#Set working directory.
wd= os.path.dirname(os.path.dirname(os.path.abspath("__file__")))
os.chdir(wd)

#Set path and directory. [Need's to be tested for portability.]
FILE_Path = 'Raw_Data/IMU.CSV'

#Read in data.
RolPitYaw_b, RolPitYaw_dt_b, Accel_b, Magnet_b,line_count = File_read(FILE_Path)

#############################################
#Calibration Stage                          #
#                                           #
#############################################
Calib_Count = 100

Calib_Z_Vector_Mag = np.zeros((Calib_Count,1))
Calib_Z_Vector_Angle = np.zeros((Calib_Count,3))
Calib_Z_Vector = np.zeros((Calib_Count,3))

for i in range(Calib_Count):
    #Determine our z location, via intial calibration. [Note: Build in failsafe, if forces being exerted during calibration]
    Calib_Z_Vector_Mag[i] = math.sqrt(Accel_b[i,0]**2 + Accel_b[i,1]**2 + Accel_b[i,2]**2)
    Calib_Z_Vector_Angle[i,...] = np.degrees(np.array([math.cos(Accel_b[i,0]/Calib_Z_Vector_Mag[i] ),math.cos(Accel_b[i,1]/Calib_Z_Vector_Mag[i] ),math.cos(Accel_b[i,2]/Calib_Z_Vector_Mag[i] )]))
    Calib_Z_Vector[i,...] = -Accel_b[i,:]/Calib_Z_Vector_Mag[i,:] 
    
Calib_Z_Vector =  np.array([np.mean(Calib_Z_Vector[:,0]),np.mean(Calib_Z_Vector[:,1]),np.mean(Calib_Z_Vector[:,2])])*np.mean(Calib_Z_Vector_Mag)*unit
Calib_Z_RolPitYaw =  np.array([np.mean(RolPitYaw_b[:,0]),np.mean(RolPitYaw_b[:,1]),np.mean(RolPitYaw_b[:,2])])
Q_Calib_Z_RolPitYaw = eul_to_quat(Calib_Z_RolPitYaw)

#############################################
#Position Estimation                        #
#                                           #
#############################################
Follow_Count = line_count - Calib_Count

Accel_n = np.zeros((Follow_Count,3))
Q_Accel_n = np.zeros((Follow_Count,3))
Q_Diff_RolPitYaw = np.zeros((Follow_Count,4))
Diff_RolPitYaw = np.zeros((Follow_Count,3))

for i in range(Follow_Count) :
    Q_Diff_RolPitYaw[i,...] = quat_prod(eul_to_quat(RolPitYaw_b[i +Calib_Count]),quat_conj(Q_Calib_Z_RolPitYaw))
    Diff_RolPitYaw[i,...] = RolPitYaw_b[i +Calib_Count] - Calib_Z_RolPitYaw 
    Q_Accel_n[i,...] = quat_Rot_By_Angle(unit*Accel_b[i +Calib_Count],Q_Diff_RolPitYaw[i,...])- Calib_Z_Vector
    Accel_n[i,...] = ROT_FRAME_CHANGE(Diff_RolPitYaw[i,...]).dot(unit*Accel_b[i +Calib_Count]) - Calib_Z_Vector*unit
    
Vel_n = np.zeros((Follow_Count,3))
Pos_n = np.zeros((Follow_Count,3))
 
t = np.array((range(Follow_Count)))
t_2 = np.array((range(line_count)))
##Butter Worth Filter
# Filter requirements.
order = 1
fs = 30      # sample rate, Hz
cutoff = .3
#Low Pass FIlter 
b, a = butter_lowpass(cutoff, fs, order)
#for i in range(3):
#    Accel_n[...,i] = butter_lowpass_filter(Accel_n[...,i], cutoff, fs, order)



for i in range(Follow_Count):
    for j in range(3):
        Vel_n[i,j] = np.trapz(Accel_n[0:i,j],dx = 1/Sample_Freq)
Vel_n[...,0] = signal.detrend(Vel_n[...,0]) 
Vel_n[...,1] = signal.detrend(Vel_n[...,1])  
Vel_n[...,2] = signal.detrend(Vel_n[...,2])   
  
  
for i in range(Follow_Count):
    for j in range(3):
        Pos_n[i,j] = np.trapz(Vel_n[0:i,j],dx = 1/Sample_Freq)  

Pos_n[...,0] = signal.detrend(Pos_n[...,0]) 
Pos_n[...,1] = signal.detrend(Pos_n[...,1])  
Pos_n[...,2] = signal.detrend(Pos_n[...,2])     

fig = plt.figure(figsize=plt.figaspect(0.5))

a_b = fig.add_subplot(1, 5, 1)
a_n = fig.add_subplot(1, 5, 2)
v_n = fig.add_subplot(1, 5, 3)
x_n = fig.add_subplot(1, 5, 4)
angle_n = fig.add_subplot(1, 5,5)
 
a_b.plot(t_2,Accel_b, label='Acceleration')   
a_n.plot(t,Accel_n, label='Acceleration')
v_n.plot(t,Vel_n, label='Velocity')
x_n.plot(t,Pos_n, label='Position')
angle_n.plot(t,Diff_RolPitYaw, label='Angle')


fig_2 = plt.figure(figsize=plt.figaspect(0.5))

x_n_2 = fig_2.add_subplot(3, 2, 1)
y_n_2 = fig_2.add_subplot(3, 2, 3)
z_n_2 = fig_2.add_subplot(3, 2, 5)
xang_n_2 = fig_2.add_subplot(3, 2, 2)
yang_n_2 = fig_2.add_subplot(3, 2, 4)
zang_n_2 = fig_2.add_subplot(3, 2, 6)

x_n_2.plot(t_2,Accel_b[:,0], label='Acceleration')   
y_n_2.plot(t_2,Accel_b[:,1], label='Acceleration')
z_n_2.plot(t_2,Accel_b[:,2], label='Velocity')
xang_n_2.plot(t_2,RolPitYaw_b[:,0],label = 'x')
yang_n_2.plot(t_2,RolPitYaw_b[:,1],label = 'y')
zang_n_2.plot(t_2,RolPitYaw_b[:,2],label = 'z')