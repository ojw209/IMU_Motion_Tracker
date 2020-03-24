'''
Auther: Oliver West - ojw209@exeter.ac.uk
Date: 20/3/2019

Function List, for reference change functions.

Let b be the body frame, n be the navigation frame and w be the world frame.
'''
import numpy as np
import math


def eul_to_quat(RolPitYaw):
        roll, pitch, yaw = RolPitYaw[:,0], RolPitYaw[:,1], RolPitYaw[:,2]

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return np.stack((qx, qy, qz, qw), axis=-1)
    

def quat_to_eul(Quat):
    x, y, z, w = Quat[...,0], Quat[...,1], Quat[...,2], Quat[...,3]
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return np.stack((roll,pitch,yaw), axis=-1)

def quat_conj(qx,qy,qz,qw):
    
    return[qx,-qy,-qz,-qw]
    
def quat_prod(quat_1, quat_0):
    w0, x0, y0, z0 = quat_1
    w1, x1, y1, z1 = quat_0
    
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0])

# https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion
def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = np.cos(theta)
    x = x * np.sin(theta)
    y = y * np.sin(theta)
    z = z * np.sin(theta)
    return w, x, y, z

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = np.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v
    
'''def quat_rot(vector,Quat):
    vector_q = np.insert(vector, 0, 0, axis=0)
    
    
    quat_prod(quat_prod(Quat,vector_q),quat_conj(quat_prod))
        #Determine our z location, via intial calibration. [Note: Build in failsafe, if forces being exerted during calibration]
    Calib_Z_Vector_Mag = math.sqrt(Accel_b[i,0]**2 + Accel_b[i,1]**2 + Accel_b[i,2]**2)
    Calib_Z_Vector_Angle = np.degrees(np.array([math.cos(Accel_b[i,0]/Calib_Z_Vector_Mag ),math.cos(Accel_b[i,1]/Calib_Z_Vector_Mag ),math.cos(Accel_b[i,2]/Calib_Z_Vector_Mag )]))
    Calib_Z_Vector = -Accel_b[i,:]/Calib_Z_Vector_Mag 
    
    #Determine North,[East = Down x Magnetic_resultant -> North = East x Down]
    Calib_Magnet_Mag = math.sqrt(Magnet_b[i,0]**2 + Magnet_b[i,1]**2 + Magnet_b[i,2]**2)
    Calib_Magnet_Vector = Magnet_b[i,:]/Calib_Magnet_Mag
    
    #Calculate reference frame.
    East_vector = np.cross(Calib_Z_Vector,Calib_Magnet_Vector)
    North_vector = np.cross(East_vector, Calib_Z_Vector)
    
    #East_vector_angle = np.degrees(np.array([math.cos(East_vector[0]),math.cos(East_vector[1]),math.cos(East_vector[2])]))
    #North_vector_angle = np.degrees(np.array([math.cos(North_vector[0]),math.cos(North_vector[1]),math.cos(North_vector[2])]))
    N_Frame_Change = R.from_euler('xyz',[East_vector,North_vector,Calib_Z_Vector])
 
    r1 = axisangle_to_q(, numpy.pi / 2)
    r2 = axisangle_to_q(y_axis_unit, numpy.pi / 2)
    r3 = axisangle_to_q(z_axis_unit, numpy.pi / 2)
    
    print(RolPitYaw_N)
    #Change to navigation frame.


'''