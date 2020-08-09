import numpy as np
'''
Resultant Force
Author: Oliver West
Date: 24/07/2020
Description: Main working script.

'''

def R_Vector(ACCEL_N,Line_count):
    
    Resultant = np.zeros(Line_count)
    Euler_Angle = np.zeros((Line_count,3))

    for i in range(Line_count):
        Resultant[i] = np.sqrt(ACCEL_N[i,0]**2 + ACCEL_N[i,0]**2 + ACCEL_N[i,2]**2)
        Euler_Angle[i,0] = np.arccos(ACCEL_N[i,0]/Resultant[i])
        
        Euler_Angle[i,1] = np.arccos([ACCEL_N[i,1]/Resultant[i]])
        
        Euler_Angle[i,2] = np.arccos([ACCEL_N[i,2]/Resultant[i]])