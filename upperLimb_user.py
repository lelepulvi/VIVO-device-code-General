## IMPORT Library ###################################################################
#####################################################################################
import math 
import time
import copy
from time import sleep, strftime, gmtime
from time import perf_counter
import sys, os
import openzen
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import serial
import string
import numpy as np
import keyboard
import threading
import pandas
import pandas as pd
from datetime import datetime
import nidaqmx
from nidaqmx.constants import Edge
from nidaqmx.constants import AcquisitionType 

from nidaqmx.stream_readers import AnalogMultiChannelReader
from nidaqmx import constants
#from nidaqmx import AnalogOutputTask

from nidaqmx.constants import (
LineGrouping)
import pickle
from datetime import datetime
import scipy.io

import multiprocessing as mp
import math
from collections import deque

## Function: record IMU data ######################################################################
 
## Function: enable IMU connection (from the Supplier's library) ##################################################
def connectSensors():
    global quit
    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializinng OpenZen library")
        quit = True
        sys.exit(1)

    sensor1MacID = "00:04:3E:86:26:E7"   # right thigh
   # sensor2MacID = "00:04:3E:4B:31:C3"   # right shank
    sensor3MacID = "00:04:3E:86:27:24"   # left thigh
    #sensor4MacID = "00:04:3E:86:27:9A"   # left shank
    #sensor5MacID = "00:04:3E:86:27:E0"   # hip

    print ("Connecting to Sensors")
    # Connect to sensor1
    
    while True:
        error, sensor1 = client.obtain_sensor_by_name("Bluetooth", sensor1MacID)
        print("Trying to connect Right Thigh...press E to quit")
        if keyboard.is_pressed("e"):
            quit = True
            sys.exit(1)
        if not error != openzen.ZenError.NoError: 
            break
    #if not error == openzen.ZenError.NoError:
    #    print ("Error connecting to right thigh", sensor1MacID)
    #    quit = True
    #    sys.exit(1)
    imu1 = sensor1.get_any_component_of_type(openzen.component_type_imu)

    # Connect to sensor2

    # Connect to sensor3
    while True:
        error, sensor3 = client.obtain_sensor_by_name("Bluetooth", sensor3MacID)
        print("Trying to connect Left Thigh...press E to quit")
        if keyboard.is_pressed("e"):
            quit = True
            sys.exit(1)
        
        if not error != openzen.ZenError.NoError:
            break
            #print ("Error connecting to left thigh", sensor3MacID)
            #quit = True
            #sys.exit(1)
    imu3 = sensor3.get_any_component_of_type(openzen.component_type_imu)


    print ("Sensors Connected")

    # Set stream frequency
    streamFreq = 100 # Hz
    error = imu1.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu1.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor1: {}".format(freq))

    error = imu3.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu3.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor3: {}".format(freq))

    # Sync sensors
    print ("Sensors sync")
    imu1.execute_property(openzen.ZenImuProperty.StartSensorSync)
#    imu2.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu3.execute_property(openzen.ZenImuProperty.StartSensorSync)
 #   imu4.execute_property(openzen.ZenImuProperty.StartSensorSync)
 #   imu5.execute_property(openzen.ZenImuProperty.StartSensorSync)
    # wait a moment for the synchronization commands to arrive
    time.sleep(3)

    # clear internal openzen event queue to remove existing data
    while True:
        zenEvent = client.poll_next_event()
        if zenEvent == None:
            break
    
    # set both sensors back to normal mode, sensor will start data streaming after these commands
    imu1.execute_property(openzen.ZenImuProperty.StopSensorSync)
  #  imu2.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu3.execute_property(openzen.ZenImuProperty.StopSensorSync)
  #  imu4.execute_property(openzen.ZenImuProperty.StopSensorSync)
  #  imu5.execute_property(openzen.ZenImuProperty.StopSensorSync)
    print("Sync completed")

    return (client, sensor1, imu1,  sensor3, imu3)

## Function: acquire IMUs (from Supplier's library) #####################################################
def data_acquisition(client, imu1,  imu3,  IMU_11,IMU_33,  IMU_1z,IMU_3z,):
    global imu1_data, imu1_dt, imu2_data, imu2_dt,imu3_data, imu3_dt,imu4_data, imu4_dt,imu5_data, imu5_dt, quit, f1,f2, f3, f4, f5
    
    global rt, rs, lt, ls, hip
    global record_start
    global s1, s2, s3, s4, s5, t1now, t2now ,t3now, t4now, t5now
    print("Data data_acquisition thread started")
    global LogFile1, LogFile2, LogFile3, LogFile4, LogFile5
    global IMU_1, IMU_2, IMU_3, IMU_4, IMU_5
    local = 0
    detected_time = perf_counter()
    
    while not quit:
        zenEvent = client.wait_for_next_event()

        # check for IMU1 event
        if zenEvent.event_type == openzen.ZenEventType.ImuData:
         if   zenEvent.sensor == imu1.sensor and \
            zenEvent.component.handle == imu1.component.handle:

            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])    # .r[0] -x, .r[2] - z
            IMU_1 = data
            IMU_11.value = data
            IMU_1z.value = copy.copy(imu_data.r[2])
            rt = data
            ts = copy.copy(imu_data.timestamp)
            if (s1 ==1 and record_start==1):
                  s1 = 0
                  t1now = imu_data.timestamp
            
           # data1 = [ts - t1now] + [perf_counter()- detected_time] + [data] 
            
        #elif zenEvent.event_type == openzen.ZenEventType.ImuData and \
         elif   zenEvent.sensor == imu3.sensor and \
            zenEvent.component.handle == imu3.component.handle:
            
            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_3 = data
            lt = data
            IMU_33.value = data
            IMU_3z.value = copy.copy(imu_data.r[2])            
            ts = copy.copy(imu_data.timestamp)
            if (s3 ==1 and record_start==1):
                  s3 = 0
                  t3now = imu_data.timestamp
           # data3 = [ts - t3now] + [perf_counter()- detected_time] + [data] 
            
        time.sleep(0.001)

    print("Data data_acquisition thread terminated")

## Function: calculate FSR period ####################################################

## Function: calculate FSR period ####################################################

arduino = serial.Serial('COM3', 115200, parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS, timeout=.1) #115200
    # command to arduino to execute the EMG and actuators
z = '<0, 0.0, 0.0, 0.0, 0.0, 0.0>'
arduino.write(bytes(z, "utf-8"))
time.sleep(2)

    ## Internal Function: set applied pressure
def apply_pressure(e, r, l, rm, lm): # e = EMG, r = right actuator 1, rm = right actuator 2, ...
       z = '<' + str(e) +',' + str(r) +',' +  str(l) +',' + str(rm) +',' + str(lm) + '>'
     #print(z)
       arduino.write(bytes(z, "utf-8"))
def pressure(theta):
    if theta <= -10:
        return 0.9
    elif theta >= 70:
        return 0.0
    else:
        return 0.9 * (70 - theta) / 80
    
def Anchor_pressure(theta):
    if theta >= 70:
        return 0.0
    else:
        return 0.15
    
if __name__ == "__main__":
    
        
    Allexit = mp.Value('i', 0) 

    
    IMU_11 = mp.Value('d', 0)      # x
    IMU_22 = mp.Value('d', 0)     
    IMU_33 = mp.Value('d', 0)
    IMU_44 = mp.Value('d', 0)
    IMU_55 = mp.Value('d', 0)

    IMU_1z = mp.Value('d', 0)     # z 
    IMU_2z = mp.Value('d', 0)     
    IMU_3z = mp.Value('d', 0)
    IMU_4z = mp.Value('d', 0)
    IMU_5z = mp.Value('d', 0)

 
    global quit
    global record_interval

    global IMU_1, IMU_2, IMU_3, IMU_4, IMU_5
    IMU_1, IMU_2, IMU_3, IMU_4, IMU_5 = 0,0,0,0,0
    # IMU_1 = 0 # IMU_2 = 0 # IMU_3 = 0 # IMU_4 = 0 # IMU_5 = 0

    global time_flag
    time_flag = 0

    global record_start
    record_start = 0

    # global fa
    # global LogFile
    global f1, f2, f3, f4, f5 # files for IMUs
    global LogFile1, LogFile2, LogFile3, LogFile4, LogFile5 # internal logs for filing
    global timestr 

    global s1, s2, s3, s4, s5, t1now, t2now, t3now, t4now, t5now
    s1, s2, s3, s4, s5 = 1,1,1,1,1
    t1now, t2now, t3now, t4now, t5now = 0,0,0,0,0


    global rt, rs, lt, ls, hip
    rt, rs, lt, ls, hip = 90,90,90,90,90
    imu1_dt   = []
    imu1_data = []
    imu2_dt   = []
    imu2_data = []
    imu3_dt   = []
    imu3_data = []
    imu4_dt   = []
    imu4_data = []
    imu5_dt   = []
    imu5_data = []

    #global buffer_in 
    r_pressed = 0
    test = 0
    firsttime = 1
    Emg_sig = 0
    c_time = 0
    record_start = 0
    imu_on = 1                
    # try = connect to all IMUs
    openzen.set_log_level(openzen.ZenLogLevel.Warning)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    LogFile = "Shoulder"+"_"+timestr+".csv"   # filename of output data file
    f1=open(LogFile,'w')
    data = ["Time"]  + ["R S Angle"] + ["R Anchor Pressure"] + ["R BAM Pressure"] + ["L S Angle"] + ["L Anchor Pressure"] + ["L BAM Pressure"] 

    for ele in data:
                                f1.write(str(ele)+',')
    f1.write('\n')
                

    try:      
        if (imu_on):
          (client, sensor1, imu1,  sensor3, imu3) = connectSensors()
          quit = False

        # If the connecton completes, ...
          data_thread = threading.Thread(target=data_acquisition, args=(client, imu1, imu3, IMU_11,IMU_33, IMU_1z, IMU_3z))
          data_thread.start()
        

        
        print("Press T to test actuation, R for recording, Q to stop a trial and E to exit")
        while True:
           
           # print('           {:.3f},     {:.3f}'.format (float(IMU_11.value),  float(IMU_33.value)), end ='\r' ) 
         #   print('     {:.3f}, {:.3f}'.format( float(IMU_5z.value), float(THETA)), end ='\r' ) 
          #  print('            
            rightA = Anchor_pressure(IMU_11.value) 
            rightB = pressure(IMU_11.value)                 

            leftA = Anchor_pressure(IMU_33.value) 
            leftB = pressure(IMU_33.value)                 
            print('           {:.3f}, {:.3f},{:.3f}, {:.3f}, {:.3f},{:.3f}'.format (float(IMU_11.value),  float(rightA), float(rightB),  float(IMU_33.value),  float(leftA), float(leftB)), end ='\r' ) 
            
            if keyboard.is_pressed('t'):
                test = 1
            
            if keyboard.is_pressed('r'):
                r_pressed = 1
                if firsttime == 1:
                    Emg_sig = 3
                    c_time = perf_counter() # record the current time from perf_counter()
                    record_interval = copy.copy(c_time)
                    firsttime = 0
                    print("\nc_time\nRecording started -- Press Q to Stop")
                record_start  = 1

            if test== 1 or r_pressed == 1:    
                apply_pressure(Emg_sig,rightA,rightB,leftA,leftB)
            if record_start  == 1:
                    data = [perf_counter() - c_time]  + [IMU_11.value] + [rightA] + [rightB] + [IMU_33.value] + [leftA] + [leftB]
            
            
                    for ele in data:
                            f1.write(str(ele)+',')
                    f1.write('\n')
    
            
            ## Exit the whole program ###################################################################            
            if keyboard.is_pressed('q') and r_pressed==1:
                               # quit = True
                                #Allexit.value = 1 # to quit all mp.process
                                mlocal = 1
                                Emg_sig = 0
                                apply_pressure(0,0,0,0,0)             
                                test = 0
                                record_start = 0
                                r_pressed = 0
                                firsttime = 1
                                print("\n Recording stopped")
                                print("\nPress R to record another trial or E to Exit")
                               


            if keyboard.is_pressed('e') and r_pressed == 0:
                                quit = True
                                Emg_sig = 0
                                apply_pressure(0,0,0,0,0)             
                                
                                test = 0
                                record_start = 0
                                r_pressed = 0
                               # Allexit.value = 1 # to quit all mp.process
                                #mlocal = 1
                                break            
    ## If IMU connection fails, it will come here               
    except KeyboardInterrupt:
        pass
    ## If IMU connection succeeds and the program runs through, 
    finally:
        quit = True
        Allexit.value = 1 # to quit all mp.process
        if (imu_on):
            data_thread.join()
            print ("\nStreaming of sensor data complete")
            client.close()  
            f1.close()

    
## SAVE FILES ###########################################################################
#########################################################################################

        
    df = pd.read_csv(LogFile)

    # Compute differences to detect time reset
    time_diff = df["Time"].diff()
    reset_indices = df.index[time_diff < -0.1].tolist()

        # Include start and end of DataFrame
    reset_indices = [0] + reset_indices + [len(df)]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Create output folder
    output_dir = f"Experiment_Shoulder_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
    for i in range(len(reset_indices) - 1):
            start_idx = reset_indices[i]
            end_idx = reset_indices[i + 1]
            trial_df = df.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"shoulder_data_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
            
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
        
   