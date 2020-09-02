import os
import numpy as np
from scipy import integrate
from scipy.signal import sosfilt,detrend
import matplotlib.pyplot as plt

import IMU_Graphing
import ROTATE
import FILTER
from FILE_IN import FILE_READ_IN
from FREQ_NORM import Freq_Norm, Y_Norm

'''
Main Execution File
Author: Oliver West
Date: 24/07/2020
Description: Main working script.

'''

plt.close('all')

#Change directory to location of script
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

Stationary_FLAG = 0
Irreg_Sampling_FLAG = 0
MAF_FLAG = 1

# %% File Read - make sure that .txt file is in same folder as working directory.

#File Name for IMU file.
FILE_NAME = "circa1mHorz20min.TXT"

#Read In From .txt File
Time_Ms, RAW_Accel_B,RAW_GYRO_B,RAW_MAG_B,RAW_POSEST,Line_count = FILE_READ_IN(FILE_NAME)

Time_s = Time_Ms/1000

# %% Function to deal with irregular sampling rate of IMU
if Irreg_Sampling_FLAG == 1:
    #Function To Normalise Frequency - Since sampling Rate irregular [Maybe some problems presented by downsampling]
    Norm_Time_s,Time_s,Norm_Freq = Freq_Norm(Time_Ms)
    
    #Map IMU measurements to the normalised Frequency.
    RAW_Accel_B,RAW_GYRO_B,RAW_MAG_B,RAW_POSEST = Y_Norm(RAW_Accel_B,RAW_GYRO_B,RAW_MAG_B,RAW_POSEST,Norm_Time_s,Time_s)
    
    Freq = Norm_Freq
    
if Irreg_Sampling_FLAG == 0:
    Freq = np.round(Line_count/max(Time_s)).squeeze().astype(int)



# %% Apply Moving Average Filter to both Pose Estimate Signal and Acceleration Signal - Effectively a low pass filter. 
# Neighbours parameter will depend on sampling rate etc.
    
IMU_Graphing.Mag_Spec_Plot(RAW_Accel_B,Freq,'Mag Spec - Accel (Prior MAF Filter)')

if MAF_FLAG == 1:
    
    Neighbours = 14
    
    RAW_Accel_B = FILTER.Moving_Average_Filter(RAW_Accel_B,Line_count,Neighbours,'Accel')
    RAW_POSEST = FILTER.Moving_Average_Filter(RAW_POSEST,Line_count,Neighbours,'Pose')
    
    IMU_Graphing.Mag_Spec_Plot(RAW_Accel_B,Freq,'Accel - Post MAF')
    IMU_Graphing.Mag_Spec_Plot(RAW_POSEST,Freq,'Pose - Post MAF')


# %% Band Pass Filter - Low pass for electrical noise - High pass for sensor drift, and Bias.
IMU_Graphing.Mag_Spec_Plot(RAW_Accel_B,Freq,'Mag Spec - Accel (Prior Filter)')
IMU_Graphing.Mag_Spec_Plot(RAW_POSEST,Freq,'Mag Spec - Pose (Prior Filter)')

# Filter requirements.
T =  max(Time_s)   # Sample Period
fs = Freq    # sample rate, Hz
accel_cutoff_low = 0.33 # attenuate all frequencies higher then this frequency
accel_cutoff_high = 0.04 # attenuate all frequencies lower then this frequency
pose_cutoff_low = 0.05 # desired cutoff frequency of the filter, Hz 
nyq = 0.5 * fs  # Nyquist Frequency
order = 2
n = int(T * fs) # total number of samples

b_low ,a_low,SOS_Low = FILTER.butter_lowpass(accel_cutoff_low,Freq,order)
b_high ,a_high ,SOS_high = FILTER.butter_highpass(accel_cutoff_high,Freq,order)

#Apply Butterworth to acceleration signal. 
#X_Filter = FILTER.butter_bandpass_filter(RAW_Accel_B[:,0],accel_cutoff_low, accel_cutoff_high, fs, order)
#Y_Filter = FILTER.butter_bandpass_filter(RAW_Accel_B[:,1],accel_cutoff_low, accel_cutoff_high, fs, order)
#Z_Filter = FILTER.butter_bandpass_filter(RAW_Accel_B[:,2],accel_cutoff_low, accel_cutoff_high, fs, order)

X_Filter = sosfilt(SOS_high, RAW_Accel_B[:,0]*10)
Y_Filter = sosfilt(SOS_high, RAW_Accel_B[:,1]*10)
Z_Filter = sosfilt(SOS_high, RAW_Accel_B[:,2]*10)


But_Accel_N = np.array((X_Filter,Y_Filter,Z_Filter)).transpose()

IMU_Graphing.Plot_Raw_Split(But_Accel_N[1000:,...],Time_s[1000:],'Filtered Accel Data - High Pass')

X_Filter = sosfilt(SOS_Low, But_Accel_N[:,0])
Y_Filter = sosfilt(SOS_Low, But_Accel_N[:,1])
Z_Filter = sosfilt(SOS_Low, But_Accel_N[:,2])


But_Accel_N = np.array((X_Filter,Y_Filter,Z_Filter)).transpose()

IMU_Graphing.Plot_Raw_Split(But_Accel_N[1000:,...],Time_s[1000:],'Filtered Accel Data - Low Pass')

IMU_Graphing.Mag_Spec_Plot(But_Accel_N, Freq,'Mag Spec - Accel (Post Filter)')


IMU_Graphing.Mag_Spec_Plot(RAW_Accel_B,Freq,'Mag Spec - Accel')
IMU_Graphing.Mag_Spec_Plot(RAW_POSEST,Freq,'Mag Spec - Pose')

# %% Rotation from the Body Frame into the Navigation frame.
But_Accel_N,RAW_POSEST = ROTATE.Rotate(But_Accel_N,RAW_POSEST,Line_count,Time_s)

# %% Trapezoidal Integration for position.
But_Vel_N = integrate.cumtrapz(But_Accel_N[1000:,...],axis = 0,dx = 1/Freq)
But_Vel_N = detrend(But_Vel_N,axis = 0)
But_Pos_N = integrate.cumtrapz(But_Vel_N,axis = 0,dx = 1/Freq)

IMU_Graphing.Plot_Raw_Split(But_Accel_N[1000:,...],Time_s[1000:],'Post Filtered Accel')
IMU_Graphing.Plot_Raw_Split(But_Vel_N,Time_s[1001:],'Post Double Integration (Pos)')
IMU_Graphing.Plot_Raw_Split(But_Pos_N,Time_s[1002:],'Post Double Integration (Pos)')