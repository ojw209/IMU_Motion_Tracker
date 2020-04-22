'''
Auther: Oliver West - ojw209@exeter.ac.uk
Date: 18/3/2019

Function List, to read in data from csv.

NOTE: GPS read in still needed.

Let b be the body frame, n be the navigation frame and w be the world frame.
'''
import csv
import numpy as np


#Read in IMU.csv file. Return's 4x[Nx3] for Orientation, Orientation.dt, Spatial acceleration and Magnetic field.
def File_read(File_Name):
    with open(File_Name) as CSV_File:  
        CSV_Data = csv.reader(CSV_File, delimiter=',')
        CSV_Rows = [row for row in CSV_Data]
        
        RolPitYaw_b = np.zeros((len(CSV_Rows),3))
        RolPitYaw_dt_b = np.zeros((len(CSV_Rows),3))
        Accel_b = np.zeros((len(CSV_Rows),3))
        Magnet_b = np.zeros((len(CSV_Rows),3))
        
        line_count = 0
        for row in CSV_Rows:
            RolPitYaw_b[line_count] = row[1:4]
            RolPitYaw_dt_b[line_count] = row[4:7]
            Accel_b[line_count] =  row[7:10]
            Magnet_b[line_count] = row[10:13]
            line_count += 1
    
    print('Read In Complete')
    return RolPitYaw_b, RolPitYaw_dt_b, Accel_b, Magnet_b,line_count
