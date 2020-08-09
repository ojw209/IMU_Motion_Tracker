import numpy as np
import csv

# %% File Read In Function 
def FILE_READ_IN(FILE_NAME):
    #Line Counter Index
    Line_count = 0
    
    #Loop to determine file Size
    with open(FILE_NAME, "r") as FILE_IN:   
        
        for LINE in FILE_IN:
            
            Line_count += 1
    
    #Intialize Empty Vectors.        
    Time_ms = np.zeros((Line_count,1))
    RAW_Accel_B = np.zeros((Line_count,3))
    RAW_GYRO_B = np.zeros((Line_count,3))
    RAW_MAG_B = np.zeros((Line_count,3))
    RAW_POSEST = np.zeros((Line_count,3))
    
    #Read In Data
    Line_count = 0
    with open(FILE_NAME, "r") as FILE_IN:
        
        CSV_IN = csv.reader(FILE_IN, delimiter=',')          
        
        for LINE in CSV_IN:
            
            #Time_ms[Line_count] = np.array(float(LINE[0]))
            RAW_Accel_B[Line_count] = np.array((float(LINE[0]),float(LINE[1]),float(LINE[2])))
            RAW_GYRO_B[Line_count] = np.array((float(LINE[3]),float(LINE[4]),float(LINE[5])))
            RAW_MAG_B[Line_count] = np.array((float(LINE[6]),float(LINE[7]),float(LINE[8])))
            RAW_POSEST[Line_count] = np.array((float(LINE[9]),float(LINE[10]),float(LINE[11])))
            
            Line_count += 1
    
    Time_total_ms = 20*60*1000
    
    Time_ms = np.linspace(0,Time_total_ms,Line_count)        
            
    
    
    return Time_ms,RAW_Accel_B,RAW_GYRO_B,RAW_MAG_B,RAW_POSEST,Line_count
