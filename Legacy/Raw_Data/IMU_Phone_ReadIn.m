Set_Fs = 100;

Sync_Data = synchronize(Acceleration,Orientation,MagneticField,AngularVelocity,'regular','linear','SampleRate',Set_Fs);

ACC_Data = timetable2table(Sync_Data(:,1:3))
ACC_Data = ACC_Data(:,2:4)
ORI_Data = timetable2table(Sync_Data(:,4:6))
ORI_Data = ORI_Data(:,2:4)
MAG_Data = timetable2table(Sync_Data(:,7:9))
MAG_Data = MAG_Data(:,2:4)
ANG_Data = timetable2table(Sync_Data(:,10:12))
ANG_Data = ANG_Data(:,2:4)

Data_Length_A = height(ANG_Data)
Data_Length_O = height(ORI_Data)

ORI_Data = ORI_Data(round(1:Data_Length_O/Data_Length_A:Data_Length_O),:)

ANG_Data.Properties.VariableNames = {'Psi_dt' 'Theta_dt' 'Phi_dt'}
ORI_Data.Properties.VariableNames = {'Psi' 'Theta' 'Phi'}
ACC_Data.Properties.VariableNames = {'X' 'Y' 'Z'}
MAG_Data.Properties.VariableNames = {'X_M' 'Y_M' 'Z_M'}
Time = array2table((1:Data_Length_A)')

Data = [ Time ORI_Data ANG_Data ACC_Data MAG_Data]

writetable(Data,'IMU_3.csv');