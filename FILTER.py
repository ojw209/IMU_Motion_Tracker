import numpy as np
import IMU_Graphing
from scipy.signal import butter, lfilter,freqs

'''
Filters
Author: Oliver West
Date: 24/07/2020
Description: Main working script.

'''

# %% Band Pass Filter - Low pass for electrical noise - High pass for sensor drift, and Bias.
def butter_bandpass_filter(data, lowcut, highcut, fs, order):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_bandpass(lowcut, highcut, fs, order):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    
    #Plot Filter Response.
    IMU_Graphing.Filter_Plot(b,a,fs,'Bandpass Butterworth')
    
    return b, a

def butter_highpass(lowcut, fs, order):
    nyq = 0.5 * fs
    low = lowcut / nyq

    sos = butter(order, low, btype='highpass',output='sos')
    b,a = butter(order, low, btype='highpass')
    #Plot Filter Response.
    IMU_Graphing.Filter_Plot(b,a,fs,'Highpass Butterworth')
    
    return b, a, sos

def butter_lowpass(lowcut, fs, order):
    nyq = 0.5 * fs
    low = lowcut / nyq

    sos = butter(order, low, btype='lowpass',output='sos')
    b,a = butter(order, low, btype='lowpass')
    #Plot Filter Response.
    IMU_Graphing.Filter_Plot(b,a,fs,'Lowpass Butterworth')
    
    return b, a, sos


# %% Moving average filter: Essentiantly a low pass filter, will be used for high frequency noise. 
def Moving_Average_Filter(RAW_B,Line_Count,Neighbours,Data_Title):
    
    #Hard Coded in moving average filter based on 3 neighbours: Needs to be made variable on neighbours (I.E nested loops).
    N_Neigh = Neighbours
    
    Temp_Series = np.zeros((Line_Count,3))
    
    #Compute for first 2n -1 neighbours. [I.E First boundary condition.]
    for i in range((N_Neigh*2)+1):
        temp = RAW_B[i,...]
        
        for j in range(N_Neigh):
            temp = temp + RAW_B[i + j + 1 ,...]
        
        for k in range(i):
            temp = temp + RAW_B[i - k ,...]
            
        Temp_Series[i,...] = temp/(N_Neigh+ i + 1)
        
        
    #Compute between first(2n-1)<n<last(2n-1)
    for i in range(Line_Count - N_Neigh):
        
        temp = RAW_B[i,...]
            
        for j in range(N_Neigh):
            temp = temp + RAW_B[i + j,...]
            temp = temp + RAW_B[i - j,...]
        
        Temp_Series[i,...] = temp/((N_Neigh*2)+1)
    
    #Compute for first 2n -1 neighbours. [I.E First boundary condition.]
    #2n-1 Line Counter:
    End_Counter = Line_Count - N_Neigh
    
    for i in range(N_Neigh):
        temp = RAW_B[i + End_Counter,...]
        
        for j in range(N_Neigh - i):
            temp = temp + RAW_B[j + End_Counter ,...]
        
        for k in range(N_Neigh):
            temp = temp + RAW_B[i - k + End_Counter ,...]
                 
        Temp_Series[i+ End_Counter,...] = temp/(2*N_Neigh - i + 1)
    
                
    IMU_Graphing.Plot_Raw(RAW_B, Data_Title + ' Data: Prior MAF' )
    IMU_Graphing.Plot_Raw(Temp_Series, Data_Title + ' Data: Post MAF')
    
    return RAW_B



    