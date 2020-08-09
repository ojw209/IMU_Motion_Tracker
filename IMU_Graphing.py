import matplotlib.pyplot as plt
from scipy.signal import freqz
from scipy import integrate
import numpy as np
import math

'''
Plot Generation File
Author: Oliver West
Date: 24/07/2020
Description: Main working script.

'''


def Plot_Raw(RAW_Data,Title):
    fig1, ax1 = plt.subplots()
    ax1.plot(RAW_Data)
    ax1.legend(('X-Axis', 'Y-Axis', 'Z-Axis'))
    ax1.grid()
    fig1.suptitle(Title, fontsize=16)
    
def Plot_Raw_Split(RAW_Data,Time,Title):
    fig, axs = plt.subplots(3, 1,sharey=True)
    axs[0].plot(Time,RAW_Data[:,0],'C0')
    axs[0].set_title('X-Axis')
    axs[1].plot(Time,RAW_Data[:,1],'C1')
    axs[1].set_title('Y-Axis')
    axs[2].plot(Time,RAW_Data[:,2],'C2')
    axs[2].set_title('Z-Axis')
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    fig.suptitle(Title, fontsize=16)
    
def G_Error_Plot(RAW_Data,ACCEL_N,G_Theory_N,G_Error):
    fig, axs = plt.subplots(1, 3)
    axs[0].plot(ACCEL_N)
    axs[0].set_title('Rotated Accel')
    axs[1].plot(G_Theory_N)
    axs[1].set_title('G Theoretical')
    axs[2].plot(G_Error)
    axs[2].set_title('Resting Error')
    
    
def Mag_Spec_Plot(T_Series,Fs,Title):
    fig, axs = plt.subplots(1,3)
    axs[0].set_title("X")
    axs[0].magnitude_spectrum(T_Series[:,0],Fs,scale = 'dB')
    axs[1].set_title("Y")
    axs[1].magnitude_spectrum(T_Series[:,1],Fs,scale = 'dB')
    axs[2].set_title("Z")
    axs[2].magnitude_spectrum(T_Series[:,2],Fs,scale = 'dB')
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    fig.suptitle(Title, fontsize=16)

def Error_Stats(DATA):
    n_bins = 20
    fig, axs = plt.subplots(1, 3)
    axs[0].set_title("Accel: X")
    axs[0].hist(DATA[...,2],bins = n_bins)
    axs[1].set_title("Accel: Y")
    axs[1].hist(DATA[...,1],bins = n_bins)
    axs[2].set_title("Accel: Z")
    axs[2].hist(DATA[...,2],bins = n_bins)
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    
    axs[0].set_xlabel('m/s')
    axs[1].set_xlabel('m/s')
    axs[2].set_xlabel('m/s')
    
    axs[0].yaxis.set_visible(False)
    axs[1].yaxis.set_visible(False)
    axs[2].yaxis.set_visible(False)
    
def Plot_Dynamics(Accel,Vel,Pos):
    fig, axs = plt.subplots(3,3,sharey=True)
    fig.suptitle('Post-Prosscesed Data', fontsize=24)
    axs[0,0].set_title("Accel:X")
    axs[0,0].plot(np.linspace(0,628,num=len(Accel[:,0])),Accel[:,0])
    axs[0,1].set_title("Accel:Y")
    axs[0,1].plot(np.linspace(0,628,num=len(Accel[:,1])),Accel[:,1])
    axs[0,2].set_title("Accel:Z")
    axs[0,2].plot(np.linspace(0,628,num=len(Accel[:,2])),Accel[:,2])
    axs[1,0].set_title("Vel:X")
    axs[1,0].plot(np.linspace(0,628,num=len(Vel[:,0])),Vel[:,0])
    axs[1,1].set_title("Vel:Y")
    axs[1,1].plot(np.linspace(0,628,num=len(Vel[:,1])),Vel[:,1])
    axs[1,2].set_title("Vel:Z")
    axs[1,2].plot(np.linspace(0,628,num=len(Vel[:,2])),Vel[:,2])
    axs[2,0].set_title("Accel:X")
    axs[2,0].plot(np.linspace(0,628,num=len(Pos[:,0])),Pos[:,0])
    axs[2,1].set_title("Accel:Y")
    axs[2,1].plot(np.linspace(0,628,num=len(Pos[:,1])),Pos[:,1])
    axs[2,2].set_title("Accel:Z")
    axs[2,2].plot(np.linspace(0,628,num=len(Pos[:,2])),Pos[:,2])
    
    
def Filter_Plot(b,a,fs,Title):
    w, h = freqz(b, a,worN = 2048)
    fig, (ax1,ax2) = plt.subplots(1,2)
    ax1.set_title('Digital filter frequency response')

    ax1.semilogx((fs*0.5/np.pi )*w, 20 * np.log10(abs(h)), 'g')
    ax1.set_ylabel('Amplitude [dB]', color='g')
    ax1.set_xlabel('Frequency (hz)')
    #ax1.set_xlim([0, 1.5])
    ax1.grid()
    

    angles = np.unwrap(np.angle(h))
    
    ax2.semilogx((fs*0.5/np.pi )*w, angles, 'b')
    ax2.set_xlabel('Frequency (hz)')
    ax2.set_ylabel('Angle (radians)', color='b')
    #ax2.set_xlim([0, 1.5])
    ax2.grid()


    plt.show()

#Plot to demonstrate the effects of Sensor Drift
def Sensor_Drif_Dem(RAW_GYRO_X):
    DATA = RAW_GYRO_X[10000:15000]
    fig, axs = plt.subplots(1)
    axs.plot(DATA,label = 'Raw Gyro Data',linewidth=0.25)
    axs.axhline(y=np.mean(DATA), color='r', linestyle='-',label = "Bias")
    Int_Pose = integrate.cumtrapz(DATA,axis = 0,dx = 0.0005)
    axs.plot(Int_Pose,'g',label = 'Pose Estimate')
    axs.legend()
    axs.set_ylabel('Angular Velocity (deg/s)')
    axs.set_xlabel('Measurements')
    fig.suptitle("Problems With Sensor Drift", fontsize=16)
    
    