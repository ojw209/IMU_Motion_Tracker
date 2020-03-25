'''
Auther: Oliver West - ojw209@exeter.ac.uk
Date: 18/3/2019

Function List, to read in data from csv.

Let b be the body frame, n be the navigation frame and w be the world frame.
'''

import matplotlib.pyplot as plt

def Angle_Plots(RolPitYaw_n,t):
    plt.close('all')
    fig = plt.figure(figsize=plt.figaspect(0.5))
    
    xx = fig.add_subplot(1, 3, 1)
    yx = fig.add_subplot(1, 3, 2)
    zx = fig.add_subplot(1, 3, 3)
    
    xx.plot(t,RolPitYaw_n[...,0], label='x-Axis')
    yx.plot(t,RolPitYaw_n[...,1], label='y-Axis')
    zx.plot(t,RolPitYaw_n[...,2], label='z-Axis')
    
    
def Position_Plots(Accel_n,Accel_n_f,Vel_n,Vel_n_f,Pos_n,Pos_n_f,t):
    fig_2 = plt.figure(figsize=plt.figaspect(0.5))
    xx = fig_2.add_subplot(2, 3, 1)
    yx = fig_2.add_subplot(2, 3, 2)
    zx = fig_2.add_subplot(2, 3, 3)  
    
    xx.plot(t,Accel_n[...,0])
    yx.plot(t,Accel_n[...,1])
    zx.plot(t,Accel_n[...,2])
    
    xx_f = fig_2.add_subplot(2, 3, 4)
    yx_f = fig_2.add_subplot(2, 3, 5)
    zx_f = fig_2.add_subplot(2, 3, 6)  
    
    xx_f.plot(t,Accel_n_f[...,0])
    yx_f.plot(t,Accel_n_f[...,1])
    zx_f.plot(t,Accel_n_f[...,2])
    
    fig_3 = plt.figure(figsize=plt.figaspect(0.5))
    xx = fig_3.add_subplot(2, 3, 1)
    yx = fig_3.add_subplot(2, 3, 2)
    zx = fig_3.add_subplot(2, 3, 3)  
    
    xx.plot(t,Vel_n[...,0])
    yx.plot(t,Vel_n[...,1])
    zx.plot(t,Vel_n[...,2])
    
    xx_f = fig_3.add_subplot(2, 3, 4)
    yx_f = fig_3.add_subplot(2, 3, 5)
    zx_f = fig_3.add_subplot(2, 3, 6)  
    
    xx_f.plot(t,Vel_n_f[...,0])
    yx_f.plot(t,Vel_n_f[...,1])
    zx_f.plot(t,Vel_n_f[...,2])
    
    fig_4 = plt.figure(figsize=plt.figaspect(0.5))
    xx = fig_4.add_subplot(2, 3, 1)
    yx = fig_4.add_subplot(2, 3, 2)
    zx = fig_4.add_subplot(2, 3, 3)  
    
    xx.plot(t,Pos_n[...,0])
    yx.plot(t,Pos_n[...,1])
    zx.plot(t,Pos_n[...,2])
    
    xx_f = fig_4.add_subplot(2, 3, 4)
    yx_f = fig_4.add_subplot(2, 3, 5)
    zx_f = fig_4.add_subplot(2, 3, 6)  
    
    xx_f.plot(t,Pos_n_f[...,0])
    yx_f.plot(t,Pos_n_f[...,1])
    zx_f.plot(t,Pos_n_f[...,2])
    
    