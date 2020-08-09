import numpy as np
import math

# %% Function to normalise time steps - some reason Arduino saves finding at irregular time steps.
def Freq_Norm(Time_Ms):
    #Calculate Time and Average Frequency
    Time_s = Time_Ms/1000
    
    Time_intervals = np.zeros((len(Time_s),1))
    for i in range(len(Time_s)):
        Time_intervals[i] = Time_s[i] - Time_s[i-1]
        
    Time_intervals[0] = Time_s[0] 
    Average_Period = np.mean(Time_intervals)
    Average_Freq = 1/Average_Period
    
    #Normalise Freqeuncy
    Norm_Freq = math.floor(Average_Freq)
    Norm_Period = 1/Norm_Freq
    Norm_Time_s = Norm_Period*np.arange(len(Time_s))
    
    return Norm_Time_s,Time_s, Norm_Freq

# %% Interpolate IMU recordings to line up with the normalised time steps.
def Y_Norm(RAW_Accel_B,RAW_GYRO_B,RAW_MAG_B,RAW_POSEST,Norm_Time_s,Time_s):
    Time_s = np.ravel(Time_s)
    Norm_Time_s = np.ravel(Norm_Time_s)
    
    Norm_Accel_B_X = np.interp(Norm_Time_s, Time_s, RAW_Accel_B[...,0])
    Norm_Accel_B_Y = np.interp(Norm_Time_s, Time_s,RAW_Accel_B[...,1])
    Norm_Accel_B_Z = np.interp(Norm_Time_s, Time_s,RAW_Accel_B[...,2])
    
    Norm_Accel_B = np.array((Norm_Accel_B_X,Norm_Accel_B_Y,Norm_Accel_B_Z)).transpose()
    
    Norm_GYRO_B_X = np.interp(Norm_Time_s, Time_s,RAW_GYRO_B[...,0])
    Norm_GYRO_B_Y = np.interp(Norm_Time_s, Time_s,RAW_GYRO_B[...,1])
    Norm_GYRO_B_Z = np.interp(Norm_Time_s, Time_s,RAW_GYRO_B[...,2])
    
    
    Norm_GYRO_B = np.array((Norm_GYRO_B_X,Norm_GYRO_B_Y,Norm_GYRO_B_Z)).transpose()
    
    Norm_MAG_B_X = np.interp(Norm_Time_s, Time_s,RAW_MAG_B[...,0])
    Norm_MAG_B_Y = np.interp(Norm_Time_s, Time_s,RAW_MAG_B[...,1])
    Norm_MAG_B_Z = np.interp(Norm_Time_s, Time_s,RAW_MAG_B[...,2])
    
    Norm_MAG_B = np.array((Norm_MAG_B_X,Norm_MAG_B_Y,Norm_MAG_B_Z)).transpose()
    
    Norm_POSEST_X = np.interp(Norm_Time_s, Time_s,RAW_POSEST[...,0])
    Norm_POSEST_Y = np.interp(Norm_Time_s, Time_s,RAW_POSEST[...,1])
    Norm_POSEST_Z = np.interp(Norm_Time_s, Time_s,RAW_POSEST[...,2])
    
    Norm_POSEST = np.array((Norm_POSEST_X,Norm_POSEST_Y,Norm_POSEST_Z)).transpose()
    
    return Norm_Accel_B,Norm_GYRO_B,Norm_MAG_B,Norm_POSEST