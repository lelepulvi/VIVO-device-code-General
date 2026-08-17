#### Adaptive Threshold #################
#### EMG reversed #######################
#### falling edge start, rising edge stop
### line 2697 and 2699 heelstrike needs to be updated

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
from multiprocessing import Value


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

from dataclasses import dataclass


@dataclass
class CPGState:

    dt: float = 0.0

    phase_right: float = 0.0
    phase_left: float = 0.0

    phase_right_deg: float = 0.0
    phase_left_deg: float = 0.0

    goal_right: float = 0.0
    goal_left: float = 0.0

    goal_right_deg: float = 0.0
    goal_left_deg: float = 0.0

    lookup_right: float = 0.0
    lookup_left: float = 0.0

    simple_right: float = 0.0
    simple_left: float = 0.0

    error_right: float = 0.0
    error_left: float = 0.0

    error_right_deg: float = 0.0
    error_left_deg: float = 0.0

    theta_tr: float = 0.0
    theta_tl: float = 0.0

    theta_er: float = 0.0
    theta_el: float = 0.0

    y_tr: float = 0.0
    y_tl: float = 0.0

    y_er: float = 0.0
    y_el: float = 0.0

    torque_right: float = 0.0
    torque_left: float = 0.0

    pressure_right: float = 0.0
    pressure_left: float = 0.0


## Function: record IMU data ######################################################################
def file_record():
#############       Recording file for analog inputs      #############
    global fa, timestr  
    timestr = time.strftime("%Y%m%d-%H%M%S")
############################     IMU files      #######################

    global f1, f2, f3, f4, f5 # files
    # timestr = time.strftime("%Y%m%d-%H%M%S")
    global LogFile1, LogFile2, LogFile3, LogFile4, LogFile5
    LogFile1 = "LP_1"+"_"+timestr+".csv"   # filename of output data file
    f1=open(LogFile1,'w')
    data = ["rtTime"]  + ["R Thigh Angle"] + ["R Thigh Angle (unwrap)"]
    for ele in data:
                        f1.write(str(ele)+',')
    f1.write('\n')
 
    LogFile2 = "LP_2"+"_"+timestr+".csv"   # filename of output data file
    f2=open(LogFile2,'w')
    data = ["rsTime"]  + ["R Shank Angle"] + ["R Shank Angle (unwrap)"] 
    for ele in data:
                        f2.write(str(ele)+',')
    f2.write('\n')
 
    LogFile3 = "LP_3"+"_"+timestr+".csv"   # filename of output data file
    f3=open(LogFile3,'w')
    data = ["ltTime"]  + ["L Thigh Angle"] + ["L Thigh Angle (unwrap)"] 
    for ele in data:
                        f3.write(str(ele)+',')
    f3.write('\n')
 
    LogFile4 = "LP_4"+"_"+timestr+".csv"   # filename of output data file
    f4=open(LogFile4,'w')
    data = ["lsTime"]  + ["L Shank Angle"] + ["L Shank Angle (unwrap)"] 
    for ele in data:
                        f4.write(str(ele)+',')
    f4.write('\n')
 
    LogFile5 = "LP_5"+"_"+timestr+".csv"   # filename of output data file
    f5=open(LogFile5,'w')
    data = ["hTime"]  + ["Hip Angle X"] + ["Hip Angle Rot"] 
    for ele in data:
                        f5.write(str(ele)+',')
    f5.write('\n')
 
## Function: enable IMU connection (from the Supplier's library) ##################################################
def connectSensors():
    global quit
    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializinng OpenZen library")
        quit = True
        sys.exit(1)

    sensor1MacID = "00:04:3E:86:26:E7"   # right thigh
    sensor2MacID = "00:04:3E:4B:31:C3"   # right shank
    sensor3MacID = "00:04:3E:86:27:24"   # left thigh
    sensor4MacID = "00:04:3E:86:27:9A"   # left shank
    sensor5MacID = "00:04:3E:86:27:E0"   # hip

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
    while True:
        error, sensor2 = client.obtain_sensor_by_name("Bluetooth", sensor2MacID)
        print("Trying to connect Right Shank...press E to quit")
        if keyboard.is_pressed("e"):
            quit = True
            sys.exit(1)
        
        
        if not error != openzen.ZenError.NoError:
            break
            #print ("Error connecting to right shank", sensor2MacID)
            #quit = True
            #sys.exit(1)
    imu2 = sensor2.get_any_component_of_type(openzen.component_type_imu)

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

    while True:
        error, sensor4 = client.obtain_sensor_by_name("Bluetooth", sensor4MacID)
        print("Trying to connect Left Shank...press E to quit")
        if keyboard.is_pressed("e"):
            quit = True
            sys.exit(1)
        if not error != openzen.ZenError.NoError:
            break
            #print ("Error connecting to left shank", sensor4MacID)
            #quit = True
            #sys.exit(1)
    imu4 = sensor4.get_any_component_of_type(openzen.component_type_imu)
    
    while True:
        error, sensor5 = client.obtain_sensor_by_name("Bluetooth", sensor5MacID)
        print("Trying to connect Hip...press E to quit")
        if keyboard.is_pressed("e"):
            quit = True
            sys.exit(1)
        if not error != openzen.ZenError.NoError:
            #print ("Error connecting to hip", sensor5MacID)
            #quit = True
            #sys.exit(1)
            break
    imu5 = sensor5.get_any_component_of_type(openzen.component_type_imu)

    print ("Sensors Connected")

    # Set stream frequency
    streamFreq = 100 # Hz
    error = imu1.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu1.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor1: {}".format(freq))

    error = imu2.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu2.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor2: {}".format(freq))

    error = imu3.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu3.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor3: {}".format(freq))

    error = imu4.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu4.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor4: {}".format(freq))
    
    error = imu5.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu5.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor5: {}".format(freq))

    # Sync sensors
    print ("Sensors sync")
    imu1.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu2.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu3.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu4.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu5.execute_property(openzen.ZenImuProperty.StartSensorSync)
    # wait a moment for the synchronization commands to arrive
    time.sleep(3)

    # clear internal openzen event queue to remove existing data
    while True:
        zenEvent = client.poll_next_event()
        if zenEvent == None:
            break
    
    # set both sensors back to normal mode, sensor will start data streaming after these commands
    imu1.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu2.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu3.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu4.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu5.execute_property(openzen.ZenImuProperty.StopSensorSync)
    print("Sync completed")

    return (client, sensor1, imu1, sensor2, imu2, sensor3, imu3, sensor4, imu4, sensor5, imu5)



imu_state = {}

def update_unwrap(imu_id, IMU_angle):

    if imu_id not in imu_state:
        imu_state[imu_id] = {
            "last_raw": IMU_angle,
            "unwrapped": IMU_angle
        }
        return IMU_angle

    last_raw = imu_state[imu_id]["last_raw"]
    unwrapped = imu_state[imu_id]["unwrapped"]

    dtheta = IMU_angle - last_raw

    if dtheta > 180:
        dtheta -= 360
    elif dtheta < -180:
        dtheta += 360

    unwrapped += dtheta

    imu_state[imu_id]["last_raw"] = IMU_angle
    imu_state[imu_id]["unwrapped"] = unwrapped

    return unwrapped


## Function: acquire IMUs (from Supplier's library) #####################################################


def data_acquisition(client, imu1, imu2, imu3, imu4, imu5, IMU_11,IMU_22,IMU_33, IMU_44, IMU_55, IMU_1z, IMU_2z,IMU_3z,IMU_4z,IMU_5z):
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
            #IMU_11.value = data
            IMU_11.value = update_unwrap("rthigh",IMU_1)
            IMU_1z.value = copy.copy(imu_data.r[2])
            rt = data
            ts = copy.copy(imu_data.timestamp)
            if (s1 ==1 and record_start==1):
                  s1 = 0
                  t1now = imu_data.timestamp
            
           # data1 = [ts - t1now] + [perf_counter()- detected_time] + [data] 
            data1 = [ts - t1now]  + [data] + [IMU_11.value]
            
            if record_start == 1:
                for ele in data1:
                        f1.write(str(ele)+',')
                f1.write('\n')
 
        # check for IMU2 event
        #elif zenEvent.event_type == openzen.ZenEventType.ImuData and \
         elif   zenEvent.sensor == imu2.sensor and \
            zenEvent.component.handle == imu2.component.handle:

            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_2 = data
           # IMU_22.value = data
            IMU_22.value = update_unwrap("rshank",IMU_2)
            IMU_2z.value = copy.copy(imu_data.r[2])            
            rs = data
            ts = copy.copy(imu_data.timestamp)
            if (s2 ==1 and record_start==1):
                  s2 = 0
                  t2now = imu_data.timestamp
            #data2 = [ts - t2now] + [perf_counter()- detected_time] + [data] 
            data2 = [ts - t2now]  + [data] + [IMU_22.value]
      
            if record_start == 1:
                for ele in data2:
                        f2.write(str(ele)+',')
                f2.write('\n')
 
        #elif zenEvent.event_type == openzen.ZenEventType.ImuData and \
         elif   zenEvent.sensor == imu3.sensor and \
            zenEvent.component.handle == imu3.component.handle:
            
            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_3 = data
            lt = data
            #IMU_33.value = data
            IMU_33.value = update_unwrap("lthigh",IMU_3)
            IMU_3z.value = copy.copy(imu_data.r[2])            
            ts = copy.copy(imu_data.timestamp)
            if (s3 ==1 and record_start==1):
                  s3 = 0
                  t3now = imu_data.timestamp
           # data3 = [ts - t3now] + [perf_counter()- detected_time] + [data] 
            data3 = [ts - t3now]  + [data] + [IMU_33.value]

            if record_start == 1:
                for ele in data3:
                        f3.write(str(ele)+',')
                f3.write('\n')
        
        #elif zenEvent.event_type == openzen.ZenEventType.ImuData and \
         elif   zenEvent.sensor == imu4.sensor and \
            zenEvent.component.handle == imu4.component.handle:
            
            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_4 = data
            ls = data
           # IMU_44.value = data
            IMU_44.value = update_unwrap("lshank",IMU_4)
            IMU_4z.value = copy.copy(imu_data.r[2])            
            ts = copy.copy(imu_data.timestamp)
            if (s4 ==1 and record_start==1):
                  s4 = 0
                  t4now = imu_data.timestamp
            #data4 = [ts - t4now] + [perf_counter()- detected_time] + [data] 
            data4 = [ts - t4now]  + [data] + [IMU_44.value]
      
            if record_start == 1:
                for ele in data4:
                        f4.write(str(ele)+',')
                f4.write('\n')
       
        #elif zenEvent.event_type == openzen.ZenEventType.ImuData and \
         elif   zenEvent.sensor == imu5.sensor and \
            zenEvent.component.handle == imu5.component.handle:

            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_5 = data
            hip = data
            #IMU_55.value = data
            IMU_55.value = update_unwrap("hip",IMU_5)
            IMU_5z.value = copy.copy(imu_data.r[2])            
            ts = copy.copy(imu_data.timestamp)
            if (s5 ==1 and record_start==1):
                  s5 = 0
                  t5now = imu_data.timestamp
           # data5 = [ts - t5now] + [perf_counter()- detected_time] + [data] 
            data5 = [ts - t5now]  + [data] + [IMU_5z.value]

            if record_start == 1:
                for ele in data5:
                        f5.write(str(ele)+',')
                f5.write('\n')
 
        time.sleep(0.001)

    print("Data data_acquisition thread terminated")

## Function: calculate FSR period ####################################################

## Function: calculate FSR period ####################################################

def Period_FSR(heelR,heelL,toeR,toeL, HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL):
    
     # Right heel
     heelcountR = 0         # Right heel strike count
     heelcountR_flag = 0    # Flag to enable state change
     heelcountTime_R =0     # exact time of strike
     Previous_heelcountTime_R = 0   
     timeHeelR = 0          # period between strike
     timeHeelR_avg = 0      # average of period between last three strikes
     heel_periods_R = deque(maxlen=3) # list for last three periods - Right heel
    
     # Left heel
     heelcountL, heelcountL_flag, heelcountTime_L, Previous_heelcountTime_L, timeHeelL, timeHeelL_avg = 0,0,0,0,0,0
     heel_periods_L = deque(maxlen=3) # list for last three periods - Left heel

     # Right toe
     toecountR, toecountTime_R, Previous_toecountTime_R, timetoeR, timetoeR_avg, forToeR = 0,0,0,0,0,0 
     toecountR_flag = 1   
     # Left toe
     toecountL, toecountTime_L, Previous_toecountTime_L, timetoeL, timetoeL_avg, forToeL = 0,0,0,0,0,0 
     toecountL_flag = 1
     
     try: # add try: to identify problem when the codes fail.
      #print("Came to FSR...", flush=True)
      while not Allexit.value:
        ## Right heel strike #########################################################
         HeelStateR = copy.copy(heelR.value)
        # print("1", flush=True)   
        # HeelStateR indicates heel is touched
         if HeelStateR == 1 and heelcountR_flag == 0:
             heelcountR_flag = 1
             heelcountTime_R = perf_counter() # stores the exact time of heel strike
             if heelcountR == 0:
                  gaitR.value = heelcountTime_R # gaitR is global variable for the exact time of heel strike
             forToeR = copy.copy(heelcountTime_R) # to use in the toe off time calculation
             heelcountR = heelcountR +1  # increase counter by 1
            # Prev_heelcountTime_R = perf_counter()
             #print("here")   
             if heelcountR> 1:
                  time_bet_heel_R = heelcountTime_R - Previous_heelcountTime_R # time between two consecutive heel strikes
                  #timeHeelR = timeHeelR + time_bet_heel_R # running sum of intervals
                  heel_periods_R.append(time_bet_heel_R) # add to the list, autmoatically removes the oldest entry if more than 3
            #### OLD METHOD ####
            #  if heelcountR == 3:
            #      timeHeelR_avg = timeHeelR/3 
            #      timeHeelR = 0
            #      heelcountR = 0 
            #      gaitR.value =  perf_counter()
            #  Previous_heelcountTime_R = heelcountTime_R

              ### NEW METHOD --- START ###
             if len(heel_periods_R) == 3: # when the length of the list reaches 3, start calculating average
                    timeHeelR_avg = sum(heel_periods_R) / 3
                    gaitR.value =  perf_counter()
             Previous_heelcountTime_R = heelcountTime_R
               ### NEW METHOD --- END ###

         if HeelStateR == 0 and heelcountR_flag == 1:
              heelcountR_flag = 0
              # timeHeel is actually the gait period
              #print("here")

         if timeHeelR_avg != 0:
              HPeriod_available_R.value = 1
              HeelRp.value = copy.copy(timeHeelR_avg)
              timeHeelR_avg = 0   

        ## Left heel strike #########################################################
         HeelStateL = copy.copy(heelL.value)
         #print("here")   
         if HeelStateL == 1 and heelcountL_flag == 0:
             heelcountL_flag = 1
             heelcountTime_L = perf_counter()
             forToeL = copy.copy(heelcountTime_L)
             heelcountL = heelcountL +1  
            # Prev_heelcountTime_R = perf_counter()
             #print("here")   
             if heelcountL> 1:
                  time_bet_heel_L = heelcountTime_L - Previous_heelcountTime_L 
                  #timeHeelL = timeHeelL + time_bet_heel_L 
                  heel_periods_L.append(time_bet_heel_L) # add to the list, autmoatically removes the oldest entry if more than 3
             #if heelcountL == 3:
              #   timeHeelL_avg = timeHeelL/3 
               #  timeHeelL = 0
                # heelcountL = 0   
             #Previous_heelcountTime_L = heelcountTime_L
             ### NEW METHOD --- START ###
             if len(heel_periods_L) == 3: # when the length of the list reaches 3, start calculating average
                    timeHeelL_avg = sum(heel_periods_L) / 3
                    gaitL.value =  perf_counter()
             Previous_heelcountTime_L = heelcountTime_L
               ### NEW METHOD --- END ###
         if HeelStateL == 0 and heelcountL_flag == 1:
              heelcountL_flag = 0
              
              #print("here")

         if timeHeelL_avg != 0:
              HPeriod_available_L.value = 1
              HeelLp.value = copy.copy(timeHeelL_avg)
              timeHeelL_avg = 0   

        ## Right toe off #########################################################
        # update time of every completed three gaits
        # timetoe is the time from the gait starts till toe off
         ToeStateR = copy.copy(toeR.value)
         #print("here")   
         if ToeStateR == 0 and toecountR_flag == 0:
             toecountR_flag = 1
             toecountTime_R = perf_counter() - forToeR
             toecountR = toecountR +1  
              
             if toecountR> 1:
                  #time_bet_toe_R = toecountTime_R - Previous_toecountTime_R 
                  timetoeR = previousTimetoeR + toecountTime_R 
             previousTimetoeR = toecountTime_R
             if toecountR == 3:
                 timetoeR_avg = timetoeR/3 
                 timetoeR = 0
                 toecountR = 0
                 previousTimetoeR = 0   
             Previous_toecountTime_R = toecountTime_R
         if ToeStateR == 1 and toecountR_flag == 1:
              toecountR_flag = 0
              
              #print("here")

         if timetoeR_avg != 0:
              TPeriod_available_R.value = 1
              ToeRp.value = copy.copy(timetoeR_avg)
              timetoeR_avg = 0              

        ## Left toe off #########################################################       
         ToeStateL = copy.copy(toeL.value)
         #print("here")   
         if ToeStateL == 0 and toecountL_flag == 0:
             toecountL_flag = 1
             toecountTime_L = perf_counter() - forToeL
             toecountL = toecountL +1  
              
             if toecountL> 1:
                  #time_bet_toe_R = toecountTime_R - Previous_toecountTime_R 
                  timetoeL = previousTimetoeL + toecountTime_L 
             previousTimetoeL = toecountTime_L
             if toecountL == 3:
                 timetoeL_avg = timetoeL/3 
                 timetoeL = 0
                 toecountL = 0
                 previousTimetoeL = 0   
             Previous_toecountTime_L = toecountTime_L
         if ToeStateL == 1 and toecountL_flag == 1:
              toecountL_flag = 0
              
              #print("here")

         if timetoeL_avg != 0:
              TPeriod_available_L.value = 1
              ToeLp.value = copy.copy(timetoeL_avg)
              timetoeL_avg = 0              
       #  print(' {:.3f},     {:.3f}'.format(float(HeelRp.value),  float(HeelLp.value)), end ='\r' ) 
              
     except Exception as e:
            import traceback
            print("FSR crashed", e , flush=True )
            traceback.print_exc()
## Function: calculates Actuation time based on Period #######################################
def Timesforwalk(HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL, Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn, rStrength, lStrength, rLength, lLength):
          rhflaglocal, rtflaglocal, sr = 0, 0, 0
          lhflaglocal, ltflaglocal, sl = 0, 0, 0

          # If Allexit.value = 0, true
          while not Allexit.value:
                   # print("2", flush=True) 
                    if HPeriod_available_R.value == 1:
                        Rhtime = copy.copy(HeelRp.value)   
                        rhflaglocal = 1 
                        HPeriod_available_R.value = 0
                    #    print("Heel period received")

                    if TPeriod_available_R.value == 1:
                        Rttime = copy.copy(ToeRp.value)
                        rtflaglocal = 1
                        TPeriod_available_R.value = 0
                    #    print("Toe period received")

                    if rhflaglocal == 1 and rtflaglocal == 1:
                        rhflaglocal = 0
                        rtflaglocal = 0
                        Rallow.value = 1
                        #GaitR = copy.copy(gaitR)r
                        #front 40-60
                        supportTimeR.value= (rStrength.value)*Rhtime #((Rttime/Rhtime) - 0.2 +r 0.2)*Rhtime # Toe off start # 0.7
                        unsupportTimeR.value= (rLength.value)*Rhtime#((Rttime/Rhtime) - 0.1)*Rhtime #90% of total period #((Rttrime/Rhtime) + 0.1)*Rhtime #0.2
                        #back 10-45
                        supportTimeRAn.value= (rStrength.value)*Rhtime #0% of total period#((Rttime/Rhtime) - 0.15)*Rhtime
                        unsupportTimeRAn.value= (rLength.value)*Rhtime#((Rttime/Rhtime) - 0.25)*Rhtime # Toe off 
                        # print(Rhtime, Rttime, supportTimeR,unsupportTimeR, supportTimeRAn,  unsupportTimeRAn )

                    if HPeriod_available_L.value == 1:
                            Lhtime = copy.copy(HeelLp.value)
                            lhflaglocal = 1 
                            HPeriod_available_L.value = 0
                          #  print("Left Heel period received")

                    if TPeriod_available_L.value == 1:
                            Lttime = copy.copy(ToeLp.value)
                            ltflaglocal = 1
                            TPeriod_available_L.value = 0
                         #   print("Left Toe period received")

                    if lhflaglocal == 1 and ltflaglocal == 1:
                            lhflaglocal = 0
                            ltflaglocal = 0
                            Lallow.value = 1
                            #GaitR = copy.copy(gaitR)
                            supportTimeL.value= (lStrength.value)*Lhtime#((Lttime/Lhtime) - 0.2 + 0.2)*Lhtime
                            unsupportTimeL.value= (lLength.value)*Lhtime#0.6*Lhtime #((Lttime/Lhtime) + 0.1)*Lhtime
                            supportTimeLAn.value= (lStrength.value)*Lhtime #((Lttime/Lhtime) - 0.15)*Lhrtime
                            unsupportTimeLAn.value= (lLength.value)*Lhtime #((Lttime/Lhtime) + 0.2)*Lhtime
                           
                           # supportTimeL.value= 0 #((Lttime/Lhtime) - 0.2 + 0.2)*Lhtime
                          #  unsupportTimeL.value= 0 #0.6*Lhtime #((Lttime/Lhtime) + 0.1)*Lhtime
                          #  supportTimeLAn.value= 0 #((Lttime/Lhtime) - 0.15)*Lhrtime
                          #  unsupportTimeLAn.value= 0 #((Lttime/Lhtime) + 0.2)*Lhtime
                           
                           
                           # print(Lhtime, Lttime, supportTimeL,unsupportTimeL, supportTimeLAn,  unsupportTimeLAn )

                    if keyboard.is_pressed('q'):
                            #GaitR = copy.copy(gaitR)
                            supportTimeL.value= 0
                            unsupportTimeL.value= 0
                            supportTimeLAn.value= 0
                            unsupportTimeLAn.value= 0
                            supportTimeR.value= 0
                            unsupportTimeR.value= 0
                            supportTimeRAn.value= 0
                            unsupportTimeRAn.value= 0 
            
## Function: execute the actuators' actuation ###########################################            

sit_memory = {
    'prev_rk': None,
    'prev_lk': None,
    'prev_rt': None,
    'prev_lt': None,
    'base_rk': None,
    'base_lk': None,
    'trend': 0,
    'cooldown': 0,
    'frame_count': 0
}

# ============================================================
# GLOBAL VARIABLES
# ============================================================

RightLeg = 0.0
LeftLeg = 0.0
RightArm = 0.0
LeftArm = 0.0

newData = False

exosuit_torque_right = 0.0
exosuit_torque_left = 0.0

rKnee = 0.0
lKnee = 0.0

rGaitTime = 1.4
lGaitTime = 1.4

heelRight = 0
heelLeft = 0

toeRight = 0
toeLeft = 0

time_seconds = 0.0
dt = 0.001

current_time_seconds = 0.0
last_time_seconds = None

previousHeelRight = 0
previousHeelLeft = 0

rightHeelStrikeTime = None
leftHeelStrikeTime = None

time_since_right_heel_strike = 0.0
time_since_left_heel_strike = 0.0

phase_right = 0.0
phase_left = 0.0

r = 0.0
R = 0.0

alpha = 350.0
beta = alpha / 4.0
alpha_g = alpha / 2.0
alpha_r = alpha / 2.0

K_tran = 75000.0

theta_tr = 0.0
theta_tl = 0

theta_er = 0.0
theta_el = 0

y_tr = 0.0
y_tl = 0.0

y_er = 0.0
y_el = 0.0

Kp = 5.0
Ki = 0.0
Kd = 0.50

error_right = 0.0
error_left = 0.0

prev_y_er = 0.0
prev_y_el = 0.0

integral = 0.0
derivative = 0.0
derivative = 0.0
DERIVATIVE_FILTER = 0.9

tau = 1.4 / (2.0 * math.pi)

RampValue = 1.0
RampRate = 1.0

requiredPressure_R = 0.0
requiredPressure_L = 0.0
# ============================================================
# PRESSURE COMMAND SMOOTHING
# ============================================================

filteredPressure_R = 0.0
filteredPressure_L = 0.0

lastPressureActiveTime_R = None
lastPressureActiveTime_L = None

# Low-pass filtering
PRESSURE_FILTER_ALPHA = 0.98

# Below this, consider pressure OFF
# Pressure units here are bar
MIN_ACTIVE_PRESSURE = 0.05      # 0.05 bar = 5 kPa

# Do not immediately drop to zero for a very short torque dip
ZERO_PRESSURE_HOLD_TIME = 0.05  # 50 ms

# Maximum pressure change rate
MAX_PRESSURE_RATE = 1.0         # bar / second



"""
gait_time = 1.4
half_gait = 0.7
pulse_width = 0.03
"""

def Activity(heelR,heelL,toeR,toeL,Allexit, Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn, R, L, RR, LL, Emg_sig, r_pressed, IMU_11, IMU_22, IMU_33, IMU_44, IMU_5z, IMU_55, Applied_pressure, Stage_of_user, rSitThigh, rSitShank, lSitThigh, lSitShank, rStandThigh, rStandShank, lStandThigh, lStandShank, psRu, psRd, psLu, psLd, UserInfoGiven,HeelRp, HeelLp, CPG_dt, CPG_goal_right, CPG_goal_left, CPG_lookup_right, CPG_lookup_left,CPG_theta_tr, CPG_theta_tl,CPG_theta_er, CPG_theta_el,CPG_y_tr, CPG_y_tl,CPG_y_er, CPG_y_el, CPG_torque_right, CPG_torque_left, CPG_actual_knee_right, CPG_actual_knee_left ,healthy_leg = "right"  ):
    
    healthy_leg = healthy_leg.strip().lower()

    if healthy_leg not in ("right", "left"):
        raise ValueError(
            "healthy_leg must be either 'right' or 'left'."
        )


   
    # ============================================================
    # Utility functions
    # ============================================================

    def degrees_to_radians(angle_degrees):
        return angle_degrees * math.pi / 180.0


    def radians_to_degrees(angle_radians):
        return angle_radians * 180.0 / math.pi


    def wrap_to_2pi(theta):
        return theta % (2.0 * math.pi)


    # ============================================================
    # LOOKUP / FORCING FUNCTIONS
    # ============================================================
    #
    # The author's explanation:
    #
    # lookup(theta)
    #     =
    # healthy knee profile(theta)
    #     -
    # simple unit-amplitude sine oscillator
    #
    # For the right profile:
    #
    # healthy first sine coefficient = -0.3408
    #
    # Subtracting sin(theta):
    #
    # -0.3408*sin(theta) - 1.0*sin(theta)
    #     =
    # -1.3408*sin(theta)
    #
    # This explains the -1.3408 coefficient in lookup_right().
    #
    # The lookup term supplies the missing shape that converts a
    # simple sine oscillator into the required knee profile.
    # ============================================================

    def lookup_right(theta):

        value = (
            0.3923
            + (-0.1083) * math.cos(theta)
            + (-1.3408) * math.sin(theta)
            + (-0.2322) * math.cos(2.0 * theta)
            + 0.1575 * math.sin(2.0 * theta)
            + 0.0193 * math.cos(3.0 * theta)
            + 0.0877 * math.sin(3.0 * theta)
            + (-0.0077) * math.cos(4.0 * theta)
            + 0.0023 * math.sin(4.0 * theta)
            + 0.0020 * math.cos(5.0 * theta)
            + 0.0147 * math.sin(5.0 * theta)
        )

        return value


    def lookup_left(theta):

        value = (
            0.3926
            + 0.0897 * math.cos(theta)
            + 1.3469 * math.sin(theta)
            + (-0.2492) * math.cos(2.0 * theta)
            + 0.1276 * math.sin(2.0 * theta)
            + (-0.0002) * math.cos(3.0 * theta)
            + (-0.0896) * math.sin(3.0 * theta)
            + (-0.0074) * math.cos(4.0 * theta)
            + 0.0001 * math.sin(4.0 * theta)
            + 0.0051 * math.cos(5.0 * theta)
            + (-0.0143) * math.sin(5.0 * theta)
        )

        return value


    # ============================================================
    # HEALTHY GOAL PROFILE
    # ============================================================
    #
    # Both legs receive their own phase.
    #
    # There is no artificial phase + pi because:
    #
    # right heel strike -> right phase = 0
    # left heel strike  -> left phase = 0
    #
    # Their real heel-strike timing naturally creates the
    # left-right phase relationship.
    # ============================================================

    def goal_function(phase):

        goal = (
            0.3923
            + (-0.1083) * math.cos(phase)
            + (-0.3408) * math.sin(phase)
            + (-0.2322) * math.cos(2.0 * phase)
            + 0.1575 * math.sin(2.0 * phase)
            + 0.0193 * math.cos(3.0 * phase)
            + 0.0877 * math.sin(3.0 * phase)
            + (-0.0077) * math.cos(4.0 * phase)
            + 0.0023 * math.sin(4.0 * phase)
            + 0.0020 * math.cos(5.0 * phase)
            + 0.0147 * math.sin(5.0 * phase)
        )

        return goal


    #def goal_function_right():

      #  return goal_function(phase_right)


    #def goal_function_left():

     #   return goal_function(phase_left)

    def goal_function_right():

        return goal_function(theta_tr)


    def goal_function_left():

         return goal_function(theta_tl)
    # ============================================================
    # HEEL-STRIKE PHASE UPDATE
    # ============================================================


    def update_heel_strike_phases(current_time):

        global previousHeelRight
        global previousHeelLeft

        global rightHeelStrikeTime
        global leftHeelStrikeTime

        global time_since_right_heel_strike
        global time_since_left_heel_strike

        global phase_right
        global phase_left

        global theta_tr
        global theta_er
        global theta_tl
        global theta_el

        # ========================================================
        # Detect heel-strike rising edges
        # ========================================================

        right_rising_edge = (
            heelRight == 1
            and previousHeelRight == 0
        )

        left_rising_edge = (
            heelLeft == 1
            and previousHeelLeft == 0
        )

        # ========================================================
        # RIGHT LEG IS HEALTHY / REFERENCE
        # ========================================================

        if healthy_leg == "right":

            # Only the healthy right heel strike resets the phases
            if right_rising_edge:

                rightHeelStrikeTime = current_time
                leftHeelStrikeTime = current_time

                time_since_right_heel_strike = 0.0
                time_since_left_heel_strike = 0.0

                # Healthy right leg starts at zero
                phase_right = 0.0

                # Left/impaired leg follows half a gait cycle later
                phase_left = math.pi

                # Reset canonical phases with the same relationship
                theta_tr = 0.0
                theta_er = 0.0

                theta_tl = math.pi
                theta_el = math.pi

            if rightHeelStrikeTime is not None:

                elapsed_time = (
                    current_time
                    - rightHeelStrikeTime
                )

                if elapsed_time < 0.0:
                    elapsed_time = 0.0

                time_since_right_heel_strike = elapsed_time
                time_since_left_heel_strike = elapsed_time

                if rGaitTime > 0.0:

                    reference_phase = (
                        2.0
                        * math.pi
                        * elapsed_time
                        / rGaitTime
                    )

                    reference_phase = wrap_to_2pi(
                        reference_phase
                    )

                    phase_right = reference_phase

                    phase_left = wrap_to_2pi(
                        reference_phase + math.pi
                    )

            else:

                time_since_right_heel_strike = 0.0
                time_since_left_heel_strike = 0.0

                phase_right = 0.0
                phase_left = math.pi

        # ========================================================
        # LEFT LEG IS HEALTHY / REFERENCE
        # ========================================================

        elif healthy_leg == "left":

            # Only the healthy left heel strike resets the phases
            if left_rising_edge:

                leftHeelStrikeTime = current_time
                rightHeelStrikeTime = current_time

                time_since_left_heel_strike = 0.0
                time_since_right_heel_strike = 0.0

                # Healthy left leg starts at zero
                phase_left = 0.0

                # Right/impaired leg follows half a gait cycle later
                phase_right = math.pi

                # Reset canonical phases with the same relationship
                theta_tl = 0.0
                theta_el = 0.0

                theta_tr = math.pi
                theta_er = math.pi

            if leftHeelStrikeTime is not None:

                elapsed_time = (
                    current_time
                    - leftHeelStrikeTime
                )

                if elapsed_time < 0.0:
                    elapsed_time = 0.0

                time_since_left_heel_strike = elapsed_time
                time_since_right_heel_strike = elapsed_time

                if lGaitTime > 0.0:

                    reference_phase = (
                        2.0
                        * math.pi
                        * elapsed_time
                        / lGaitTime
                    )

                    reference_phase = wrap_to_2pi(
                        reference_phase
                    )

                    phase_left = reference_phase

                    phase_right = wrap_to_2pi(
                        reference_phase + math.pi
                    )

            else:

                time_since_left_heel_strike = 0.0
                time_since_right_heel_strike = 0.0

                phase_left = 0.0
                phase_right = math.pi

        # Save heel states for rising-edge detection
        previousHeelRight = int(heelRight)
        previousHeelLeft = int(heelLeft)


    
    # ============================================================
    # PID FUNCTIONS
    # ============================================================

    def dIntegral_dt(e):

        return e


    def dDerivative_dt(error, prev_error):

        if dt <= 0.0:
            return 0.0

        return (error - prev_error) / dt


    def PID(error, prev_error):

        global integral
        global derivative

        # ========================================================
        # RK4 integral
        # ========================================================

        k1I = dt * dIntegral_dt(error)

        k2I = dt * dIntegral_dt(error + k1I / 2.0)

        k3I = dt * dIntegral_dt(error + k2I / 2.0)

        k4I = dt * dIntegral_dt( error + k3I)

        integral += (k1I + 2.0 * k2I + 2.0 * k3I + k4I) / 6.0

        # ========================================================
        # Derivative
        # ========================================================

        raw_derivative = dDerivative_dt(
             error,
             prev_error
            )

        derivative = (
         DERIVATIVE_FILTER * derivative
            + (1.0 - DERIVATIVE_FILTER) * raw_derivative
            )

        # ========================================================
        # PID output
        # ========================================================

        return (Kp * error + Ki * integral   + Kd * derivative)


    # ============================================================
    # CANONICAL SYSTEM
    # ============================================================

    def can_coupling(time_seconds, theta_i, theta_j, theta_k, K_can_1, K_can_2):

        return ( (1.0 / tau) + K_can_1  * math.sin(theta_j - theta_i) - K_can_2 * math.sin(theta_k - theta_i))


    def canonical_terms( theta_i, theta_j, theta_k, K_can_1, K_can_2 ):

        c1 = dt * can_coupling(time_seconds, theta_i, theta_j, theta_k, K_can_1, K_can_2)

        c2 = dt * can_coupling( time_seconds + dt / 2.0, theta_i + c1 / 2.0, theta_j, theta_k, K_can_1, K_can_2)

        c3 = dt * can_coupling(time_seconds + dt / 2.0, theta_i + c2 / 2.0, theta_j, theta_k, K_can_1, K_can_2)

        c4 = dt * can_coupling(time_seconds + dt, theta_i + c3, theta_j, theta_k, K_can_1, K_can_2)

        theta_i += ( c1 + 2.0 * c2 + 2.0 * c3 + c4 ) / 6.0

        return theta_i


    # ============================================================
    # TRANSFORMATION SYSTEM
    # ============================================================

    def dr_dt( time_seconds, r_value, R_value):

        return (alpha_r / tau * (R_value - r_value))


    def dy_dt(time_seconds,y, z):

        return z / tau


    def dz_dt(time_seconds,y, z,goal,error,theta_i,side):

        global r
        global R

        # ========================================================
        # The lookup function uses the same phase as the
        # corresponding goal and measured knee profile.
        # ========================================================
        if side == 1:

            R = lookup_right(theta_i)

        elif side == 2:

            R = lookup_right(theta_i)

        # ========================================================
        # RK4 update of r
        # ========================================================

        r1 = dt * dr_dt( time_seconds, r, R)

        r2 = dt * dr_dt(time_seconds + dt / 2.0, r + r1 / 2.0, R)

        r3 = dt * dr_dt( time_seconds + dt / 2.0, r + r2 / 2.0, R)

        r4 = dt * dr_dt( time_seconds + dt, r + r3, R )

        r += ( r1 + 2.0 * r2 + 2.0 * r3 + r4 ) / 6.0

        return ((alpha / tau) * ( beta * (goal - y) - z) + ( r + K_tran * error) / tau)


    def transformational_terms( goal, error, theta_i, y, side):

        # z intentionally resets every call
        z = 0.0

        # ========================================================
        # RK4 for z
        # ========================================================

        t1z = dt * dz_dt(time_seconds, y, z, goal, error, theta_i, side)

        t2z = dt * dz_dt(time_seconds + dt / 2.0, y, z + t1z / 2.0, goal, error, theta_i, side)

        t3z = dt * dz_dt( time_seconds + dt / 2.0, y, z + t2z / 2.0, goal, error, theta_i, side)

        t4z = dt * dz_dt(time_seconds + dt, y, z + t3z, goal, error, theta_i, side)

        z += (t1z + 2.0 * t2z + 2.0 * t3z + t4z ) / 6.0

        # ========================================================
        # RK4 for y
        # ========================================================

        t1y = dt * dy_dt( time_seconds, y, z)

        t2y = dt * dy_dt(time_seconds + dt / 2.0, y + t1y / 2.0, z)

        t3y = dt * dy_dt(time_seconds + dt / 2.0, y + t2y / 2.0, z)

        t4y = dt * dy_dt(time_seconds + dt, y + t3y, z)

        y += (t1y + 2.0 * t2y + 2.0 * t3y + t4y ) / 6.0

        return y


    # ============================================================
    # EXOSUIT CONTROLLER
    # ============================================================

    def exosuit():

        global exosuit_torque_right
        global exosuit_torque_left

        exosuit_torque_right = PID(y_er, prev_y_er)
        exosuit_torque_left = PID(y_el, prev_y_el)

        exosuit_torque_right *= RampValue
        exosuit_torque_left *= RampValue

        # Safety saturation
        MAX_TORQUE = 10.0

        exosuit_torque_right = max(
            -MAX_TORQUE,
            min(MAX_TORQUE, exosuit_torque_right)
        )

        exosuit_torque_left = max(
            -MAX_TORQUE,
            min(MAX_TORQUE, exosuit_torque_left)
)
    # ============================================================
    # CPG
    # Original sequence retained
    # ============================================================

    def CPG():

        global error_right
        global error_left

        global theta_tl
        global theta_tr
        global theta_el
        global theta_er

        global y_tr
        global y_er
        global y_tl
        global y_el

        global prev_y_er
        global prev_y_el

        global tau

        # ========================================================
        # Healthy-impaired angle errors
        # ========================================================

        error_right = goal_function_right()- rKnee
        error_left = goal_function_left()- lKnee
        

        # ========================================================
        # Canonical terms
        # Same original calculation order
        # ========================================================

        # Left target
        if lGaitTime > 0.0:
            tau = ( lGaitTime / (2.0 * math.pi))

        next_theta_tl = canonical_terms( theta_tl,theta_tl, theta_tr, 0, 10)

        # Right target
        if rGaitTime > 0.0:
            tau = (rGaitTime / (2.0 * math.pi))

        next_theta_tr = canonical_terms(theta_tr, theta_tr, theta_tl, 0, 10)

        # Right exosuit
        if rGaitTime > 0.0:
            tau = (rGaitTime / (2.0 * math.pi))

        next_theta_er = canonical_terms( theta_er, theta_tr, theta_el, 10, 10)

        # Left exosuit
        if lGaitTime > 0.0:
            tau = (lGaitTime/ (2.0 * math.pi))

        next_theta_el = canonical_terms(theta_el, theta_tl, theta_er, 10, 10)

        # ========================================================
        # Transformation terms
        # Same original calculation order
        # ========================================================

        # Right target trajectory
        if rGaitTime > 0.0:
            tau = (rGaitTime/ (2.0 * math.pi))

        y_tr = transformational_terms(
             goal_function_right(),
                    0.0,
                    theta_tr,
                     y_tr,
                      1
                        )
        # Right exosuit compensation
        if rGaitTime > 0.0:
            tau = ( rGaitTime/ (2.0 * math.pi))
        y_er = transformational_terms(
                    0.0,
                    error_right,
                    theta_er,
                    y_er,
                    1
                )
        # Left target trajectory
        if lGaitTime > 0.0:
            tau = (lGaitTime / (2.0 * math.pi))
        y_tl = transformational_terms(
            goal_function_left(),
            0.0,
            theta_tl,
            y_tl,
            2
        )
        # Left exosuit compensation
        if lGaitTime > 0.0:
            tau = (lGaitTime / (2.0 * math.pi))

        y_el = transformational_terms(
                0.0,
                error_left,
                theta_el,
                y_el,
                2
            )
        # ========================================================
        # Simultaneous canonical update
        # ========================================================

        theta_tl = wrap_to_2pi(next_theta_tl)

        theta_tr = wrap_to_2pi(next_theta_tr)

        theta_el = wrap_to_2pi(next_theta_el)

        theta_er = wrap_to_2pi( next_theta_er)

        # ========================================================
        # Generate exosuit torque
        # ========================================================

        exosuit()

        # ========================================================
        # Save previous exosuit outputs
        # ========================================================

        prev_y_er = y_er
        prev_y_el = y_el


    # ============================================================
    # TORQUE-TO-PRESSURE CONVERSION
    # ============================================================

    def pressureFromTorque(torque):
        if not math.isfinite(torque):
             return 0.0
        pressure = [
            0.00,
            0.10,
            0.20,
            0.30,
            0.40,
            0.50,
            0.60,
            0.70,
            0.80,
            0.90
        ]

        torqueTbl = [
            0.66,
            5.40,
            9.96,
            14.22,
            18.24,
            22.08,
            25.62,
            28.92,
            31.98,
            34.80
        ]

        N = len(pressure)

        if torque <= torqueTbl[0]:
            return pressure[0]

        if torque >= torqueTbl[N - 1]:
            return pressure[N - 1]

        for i in range(N - 1):

            if (
                torque >= torqueTbl[i]
                and torque <= torqueTbl[i + 1]
            ):

                ratio = (
                    torque
                    - torqueTbl[i]
                ) / (
                    torqueTbl[i + 1]
                    - torqueTbl[i]
                )

                return (
                    pressure[i]
                    + ratio
                    * (
                        pressure[i + 1]
                        - pressure[i]
                    )
                )

        return -1.0


    # ============================================================
    # COMPLETE CONTROLLER UPDATE
    # ============================================================

    def update_cpg_controller(
        heel_right,
        heel_left,
        HeelRp,
        HeelLp,
        right_knee_degrees,
        left_knee_degrees,
        current_time=None
    ):

        global heelRight
        global heelLeft

        global rGaitTime
        global lGaitTime

        global rKnee
        global lKnee

        global time_seconds
        global current_time_seconds
        global last_time_seconds
        global dt

        global requiredPressure_R
        global requiredPressure_L
        global filteredPressure_R
        global filteredPressure_L

        global lastPressureActiveTime_R
        global lastPressureActiveTime_L

        if current_time is None:
            current_time = perf_counter()

        current_time_seconds = current_time


        # ========================================================
        # Calculate controller dt
        # ========================================================

        if last_time_seconds is None:

            raw_dt = 0.001

        else:

            raw_dt = (
                current_time_seconds
                - last_time_seconds
            )

        # Protect against zero/negative timing
        if raw_dt <= 0.0:
            raw_dt = 0.001

        # ========================================================
        # CPG numerical integration timestep
        #
        # Do not allow RK4 to use a step larger than 1 ms.
        # This prevents a delayed multiprocessing iteration
        # from producing a very large integration step.
        # ========================================================

        dt = min(raw_dt, 0.001)

        last_time_seconds = current_time_seconds
        time_seconds = current_time_seconds
        # ========================================================
        # Update heel states
        # ========================================================

        heelRight = int(heel_right)

        heelLeft = int(heel_left)

        # ========================================================
        # Update gait periods
        # ========================================================

                # ========================================================
        # Select the healthy/reference leg's gait time
        # ========================================================

        if healthy_leg == "right":

            target_gait_time = float(HeelRp)

            # Accept only plausible/stable gait periods
            if 0.8 <= target_gait_time <= 3.0:

                rGaitTime = target_gait_time
                lGaitTime = target_gait_time

        elif healthy_leg == "left":

            target_gait_time = float(HeelLp)

            if 0.8 <= target_gait_time <= 3.0:

                rGaitTime = target_gait_time
                lGaitTime = target_gait_time
        # ========================================================
        # Convert measured knee angles to radians
        # ========================================================

        rKnee = degrees_to_radians(right_knee_degrees)

        lKnee = degrees_to_radians(left_knee_degrees)

        # ========================================================
        # Reset/update phases from real heel strikes
        # ========================================================

        update_heel_strike_phases(current_time_seconds)

        # ========================================================
        # Run CPG
        # ========================================================

        CPG()

        # ========================================================
        # Convert torque to pressure
        # ========================================================

        # ============================================================
        # RAW TORQUE -> PRESSURE
        # ============================================================

        print(
            f"Torque before pressure: "
            f"R={exosuit_torque_right:.3f}, "
            f"L={exosuit_torque_left:.3f}",
            end="\r",
            flush=True
        )

        rawPressure_R = pressureFromTorque(
            max(0.0, exosuit_torque_right)
        )

        rawPressure_L = pressureFromTorque(
            max(0.0, exosuit_torque_left)
        )


        # ============================================================
        # SAFETY AGAINST NaN / INF
        # ============================================================

        if not math.isfinite(rawPressure_R):
            rawPressure_R = 0.0

        if not math.isfinite(rawPressure_L):
            rawPressure_L = 0.0


        # ============================================================
        # LOW-PASS FILTER PRESSURE
        # ============================================================

        filteredPressure_R = (
            PRESSURE_FILTER_ALPHA * filteredPressure_R
            +
            (1.0 - PRESSURE_FILTER_ALPHA) * rawPressure_R
        )

        filteredPressure_L = (
            PRESSURE_FILTER_ALPHA * filteredPressure_L
            +
            (1.0 - PRESSURE_FILTER_ALPHA) * rawPressure_L
        )


        # ============================================================
        # ZERO-PRESSURE HOLD / DEBOUNCE
        #
        # Prevent:
        # 20 kPa -> 0 -> 20 -> 0
        # within only a few milliseconds.
        # ============================================================

        if filteredPressure_R >= MIN_ACTIVE_PRESSURE:

            targetPressure_R = filteredPressure_R
            lastPressureActiveTime_R = current_time_seconds

        else:

            if (
                lastPressureActiveTime_R is not None
                and
                current_time_seconds - lastPressureActiveTime_R
                < ZERO_PRESSURE_HOLD_TIME
            ):
                # Keep the previous pressure command briefly
                targetPressure_R = requiredPressure_R

            else:
                targetPressure_R = 0.0


        if filteredPressure_L >= MIN_ACTIVE_PRESSURE:

            targetPressure_L = filteredPressure_L
            lastPressureActiveTime_L = current_time_seconds

        else:

            if (
                lastPressureActiveTime_L is not None
                and
                current_time_seconds - lastPressureActiveTime_L
                < ZERO_PRESSURE_HOLD_TIME
            ):
                targetPressure_L = requiredPressure_L

            else:
                targetPressure_L = 0.0


        # ============================================================
        # PRESSURE RATE LIMIT
        #
        # Maximum change depends on actual controller dt.
        # MAX_PRESSURE_RATE = 1 bar/s
        #
        # At dt = 0.001:
        # maximum change = 0.001 bar = 1 kPa per iteration
        # ============================================================

        max_change = MAX_PRESSURE_RATE * dt


        delta_R = (
            targetPressure_R
            - requiredPressure_R
        )

        delta_L = (
            targetPressure_L
            - requiredPressure_L
        )


        delta_R = max(
            -max_change,
            min(max_change, delta_R)
        )

        delta_L = max(
            -max_change,
            min(max_change, delta_L)
        )


        requiredPressure_R += delta_R
        requiredPressure_L += delta_L
        # ========================================================
        # Return useful controller values
        # ========================================================
        state = CPGState()

        state.dt = dt

        state.phase_right = phase_right
        state.phase_left = phase_left

        state.phase_right_deg = radians_to_degrees(phase_right)
        state.phase_left_deg = radians_to_degrees(phase_left)

        state.goal_right = goal_function_right()
        state.goal_left = goal_function_left()

        state.goal_right_deg = radians_to_degrees(state.goal_right)
        state.goal_left_deg = radians_to_degrees(state.goal_left)

        state.lookup_right = lookup_right(theta_tr)
        state.lookup_left = lookup_left(theta_tl)

        state.simple_right = math.sin(phase_right)
        state.simple_left = math.sin(phase_left)

        state.error_right = error_right
        state.error_left = error_left

        state.error_right_deg = radians_to_degrees(error_right)
        state.error_left_deg = radians_to_degrees(error_left)

        state.theta_tr = theta_tr
        state.theta_tl = theta_tl

        state.theta_er = theta_er
        state.theta_el = theta_el

        state.y_tr = y_tr
        state.y_tl = y_tl

        state.y_er = y_er
        state.y_el = y_el

        state.torque_right = exosuit_torque_right
        state.torque_left = exosuit_torque_left

        state.pressure_right = requiredPressure_R
        state.pressure_left = requiredPressure_L

        return state
        
    
    while not Allexit.value :
                    
        right_knee_degrees = (IMU_22.value- IMU_11.value)

        left_knee_degrees = (IMU_44.value- IMU_33.value)
        
        CPG_actual_knee_right.value = right_knee_degrees
        CPG_actual_knee_left.value =   left_knee_degrees

        state = update_cpg_controller(
            heel_right=heelR.value,
            heel_left=heelL.value,
            HeelRp=HeelRp.value,
            HeelLp=HeelLp.value,
            right_knee_degrees=right_knee_degrees,
            left_knee_degrees=left_knee_degrees
        )

        
        
        """
        if Rallow.value == 1:
                            # IMPORTANT: assign the returned tuple back!
                            last_heelR, gaittimeR = process_leg(
                                heelR.value, last_heelR, gaittimeR,
                                supportTimeR, unsupportTimeR,
                                R, RR, level=pressure, side='R'
                            )

        if Lallow.value == 1:
                            last_heelL, gaittimeL = process_leg(
                                heelL.value, last_heelL, gaittimeL,
                                supportTimeL, unsupportTimeL,
                                L, LL, level=pressure, side='L'
                            )
                    #  print(pressure, flush=True)
                        # APPLY once per loop with the current actuator setpoints
        
        """


       # if r_pressed.value == 1:
                            #apply_pressure(Emg_sig.value, R.value, L.value, RR.value, LL.value)
                            #print("Walking...")
                      #  Emg_sig.value = 3
        
        
        RR.value = state.pressure_right
        R.value  = state.pressure_right

        LL.value = state.pressure_left
        L.value  = state.pressure_left
        
        CPG_dt.value  = state.dt


        CPG_goal_right.value = state.goal_right
        CPG_goal_left.value = state.goal_left

        CPG_lookup_right.value = state.lookup_right
        CPG_lookup_left.value = state.lookup_left

        CPG_theta_tr.value = state.theta_tr
        CPG_theta_tl.value = state.theta_tl

        CPG_theta_er.value = state.theta_er
        CPG_theta_el.value = state.theta_el

        CPG_y_tr.value = state.y_tr
        CPG_y_tl.value = state.y_tl

        CPG_y_er.value = state.y_er
        CPG_y_el.value = state.y_el
        CPG_torque_right.value = state.torque_right
        CPG_torque_left.value = state.torque_left
        
                           #
                           #  apply_pressure(Emg_sig.value , 0, 0, 0, 0)
        
        

        
    # Safety shutoff after loop ends


## Function: start recording DAQ ###################################################
def record_daq(Allexit, record_start_fd, c_time, LogFile, Lr, Ll, psRu, psRd, psLu,psLd, FSRrh, FSRrt, FSRlh, FSRlt, switch, heelR,heelL,HeelRp, HeelLp,toeR,toeL, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, IMU_11, IMU_22, IMU_33, IMU_44, IMU_55, IMU_1z, IMU_2z, IMU_3z, IMU_4z, IMU_5z, rSitThigh, rSitShank, lSitThigh, lSitShank, hipSit,  rStandThigh,  rStandShank, lStandThigh,  lStandShank,  hipStand,   rStrength,  lStrength, rLength, lLength, R, L, RR, LL, Emg_sig,CPG_dt, CPG_goal_right, CPG_goal_left, CPG_lookup_right, CPG_lookup_left,CPG_theta_tr, CPG_theta_tl,CPG_theta_er, CPG_theta_el,CPG_y_tr, CPG_y_tl,CPG_y_er, CPG_y_el, CPG_torque_right, CPG_torque_left   
):      # varaibles need to change
    
    ## Convert funtion: Pressure sensors ###########################################
    def Volt_to_kPa(v):
         kPa = 73.3102*v - 36.7266
         return kPa
    ## Convert funtion: Load cell ##################################################
    def Volt_to_newton(v):
         Newton = 57.8137*v - 19.9659
         return Newton
    
    # DAQ initialisation (run once)
    n = 5 # n = 5 lab, 1 at home
    taskAI = nidaqmx.Task()
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai0".format(n), min_val=0, max_val= 5)    # Loadcell right
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai1".format(n), min_val=0, max_val= 5)    # Loadcell left
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai2".format(n), min_val=0, max_val= 5)    # Pressure sensor right up
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai3".format(n), min_val=0, max_val= 5)    # Pressure sensor right down
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai4".format(n), min_val=0, max_val= 5)    # Pressure sensor left up
    
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai5".format(n), min_val=0, max_val= 5)    # Pressure sensor left down
    
    
    
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai6".format(n), min_val=0, max_val= 5)    # FSR left heel
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai7".format(n), min_val=0, max_val= 5)    # FSR left toe
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai16".format(n), min_val=0, max_val= 5)   # FSR left heel
    taskAI.ai_channels.add_ai_voltage_chan("Dev{}/ai17".format(n), min_val=0, max_val= 5)   # FSR left toe
    taskAI.timing.cfg_samp_clk_timing(1000, source="OnboardClock", active_edge=Edge.RISING, sample_mode=AcquisitionType.CONTINUOUS,samps_per_chan= 2)  # samps_per_chan= 100
            #10 works, 13 works
    taskAI.in_stream.input_buf_size = 120000

    # Start DAQ
    taskAI.start()   
    fa=open(LogFile,'w')
    data = ["Time"]  + ["R Loadcell"] + ["L Loadcell"]  + ["Given Pressure R BAM UP"] + ["Given Pressure R BAM Down"] + ["Given Pressure L BAM UP"] + ["Given Pressure L BAM Down"] + ["Pressure R BAM UP"] + ["Pressure R BAM Down"] + ["Pressure L BAM UP"] + ["Pressure L BAM Down"] + ["Emg Signal"]+ ["FSR R Heel"] + ["FSR R Toe"] + ["FSR L Heel"] + ["FSR L Toe"] + ["heelR"] + ["heelL"] +["HeelRp"] + ["HeelLp"] +["toeR"]+["toeL"]+ ["supportTimeR"] + ["unsupportTimeR"]+ ["supportTimeL"]+ ["unsupportTimeL"] + ["R thigh"] + ["R shank"] + ["L thigh"] + ["L shank"] +  ["R knee"] + ["L knee"] + ["hip x"] + ["hip r"]  + ["rThighSit"] + ["rShankSit"] + ["lThighSit"] + ["lShankSit"] +["hipSit"] + ["rThighStand"] + ["rShankStand"] + ["lThighStand"] + ["lShankStand"] +["hipStand"]  + ["Rstrength"] + ["Lstrength"]  + ["Rlength"] + ["Llength"] + ["CPG_dt"] + ["CPG_goal_right"] + ["CPG_goal_left"] + ["CPG_lookup_right"] + ["CPG_lookup_left"] +["CPG_theta_tr"] + ["CPG_theta_tl"] + ["CPG_theta_er"] + ["CPG_theta_el"] +["CPG_y_tr"] + ["CPG_y_tl"]  + ["CPG_y_er"] + ["CPG_y_el"] + ["CPG_torque_right"] + ["CPG_torque_left"]   
            
    for ele in data:
        fa.write(str(ele)+',')
    fa.write('\n')
     
    record_interval = 0
    # Read DAQ ################################################################
    
    
    while not Allexit.value:
        
        Analogs = taskAI.read()
            #Analogs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
        valuex = copy.copy(Analogs)
        Lr.value           = Volt_to_newton(valuex[0])
        Ll.value           = Volt_to_newton(valuex[1])
        psRu.value         = Volt_to_kPa(valuex[2])
        psRd.value         = Volt_to_kPa(valuex[3])
        psLu.value         = Volt_to_kPa(valuex[4])
        psLd.value         = Volt_to_kPa(valuex[5])
        if switch.value == 0:
            FSRrh.value        = valuex[6]
            FSRrt.value        = valuex[7]
            FSRlh.value        = valuex[8]
            FSRlt.value        = valuex[9]

        
        # Record data to the file
        if record_start_fd.value == 1:                      
           # if perf_counter() - record_interval > 0.009:        
                # print("here")
                dataA = [perf_counter() - c_time.value] +  [Lr.value] +  [Ll.value]  + [R.value] + [RR.value]+   [L.value] + [LL.value]+ [psRu.value] + [psRd.value] + [psLu.value] + [psLd.value] +  [Emg_sig.value] +[FSRrh.value] + [FSRrt.value] + [FSRlh.value] + [FSRlt.value]  + [heelR.value] + [heelL.value] +[HeelRp.value] + [HeelLp.value] +[toeR.value]+[toeL.value]+ [supportTimeR.value] + [unsupportTimeR.value]+ [supportTimeL.value]+ [unsupportTimeL.value] + [IMU_11.value] + [IMU_22.value] + [IMU_33.value] + [IMU_44.value] + [IMU_22.value - IMU_11.value] + [IMU_44.value - IMU_33.value] + [IMU_55.value] + [IMU_5z.value]  + [rSitThigh.value] + [rSitShank.value] + [lSitThigh.value] + [lSitShank.value] + [hipSit.value] +[rStandThigh.value] + [rStandShank.value] + [lStandThigh.value] + [lStandShank.value]  + [hipStand.value]  + [rStrength.value] + [lStrength.value] + [rLength.value] + [lLength.value] + [CPG_dt.value] + [CPG_goal_right.value] + [CPG_goal_left.value] + [CPG_lookup_right.value] + [CPG_lookup_left.value] +[CPG_theta_tr.value] + [CPG_theta_tl.value] + [CPG_theta_er.value] + [CPG_theta_el.value] +[CPG_y_tr.value] + [CPG_y_tl.value]  + [CPG_y_er.value] + [CPG_y_el.value] + [CPG_torque_right.value] + [CPG_torque_left.value]
                for ele in dataA:
                    fa.write(str(ele)+',')
                fa.write('\n')

                record_interval = perf_counter()
  
    fa.close()
    taskAI.stop()
    taskAI.close()




def Pressure_foot(Allexit, FSRrh, FSRrt, FSRlh, FSRlt):

    Pressure_serialL = serial.Serial(
        'COM14',  #COM59-- white, COM14---Black
        460800,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.005
    )

    time.sleep(2)

    Pressure_serialR = serial.Serial(
        'COM3',  # COM61-- white    COM3 -- Black
        460800,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.005
    )

    time.sleep(2)

    Pressure_serialL.reset_input_buffer()
    Pressure_serialR.reset_input_buffer()

    P1 = P2 = P3 = P4 = 0.0

    def parse_pressure_line(line):
        try:
            parts = line.split()
            values = {}

            for p in parts:
                if ":" in p:
                    key, val = p.split(":")
                    values[key] = float(val)

            return values

        except Exception as e:
            print(f"Parse error: {e}, line={line}")
            return None

    def read_latest(serial_port):
        latest_line = None

        # Read all available lines and keep only the newest complete one
        while serial_port.in_waiting > 0:
            try:
                line = serial_port.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    latest_line = line
            except Exception:
                pass

        return latest_line

    while not Allexit.value:

        # -------------------------------
        # LEFT foot: COM14
        # -------------------------------
        lineL = read_latest(Pressure_serialL)

        if lineL:
            valuesL = parse_pressure_line(lineL)

            if valuesL is not None:
                if "P1" in valuesL:
                    P3 = valuesL["P1"]   # left heel
                if "P2" in valuesL:
                    P4 = valuesL["P2"]   # left toe

        # -------------------------------
        # RIGHT foot: COM3
        # -------------------------------
        lineR = read_latest(Pressure_serialR)

        if lineR:
            valuesR = parse_pressure_line(lineR)

            if valuesR is not None:
                if "P1" in valuesR:
                    P1 = valuesR["P1"]   # right heel
                if "P2" in valuesR:
                    P2 = valuesR["P2"]   # right toe

        FSRrh.value = P1
        FSRrt.value = P2
        FSRlh.value = P3
        FSRlt.value = P4

    Pressure_serialL.close()
    Pressure_serialR.close()



class AdaptiveFSR:
    def __init__(self,
                 ema_alpha=0.2, hysteresis_frac=0.10,
                 min_contact_ms=60, min_release_ms=60, min_dwell_ms=120,
                 clip_band_min=None, clip_band_max=1e6, learn_window_steps=6):
        self.alpha = ema_alpha
        self.hfrac = hysteresis_frac
        self.min_contact = min_contact_ms / 1000.0
        self.min_release = min_release_ms / 1000.0
        self.min_dwell   = min_dwell_ms   / 1000.0

        # Will auto-choose later if None
        self.clip_band_min = clip_band_min
        self.clip_band_max = clip_band_max

        self.contact_level   = None
        self.nocontact_level = None
        self.invert = None   # auto-detect later

        self.curr_min = math.inf
        self.curr_max = -math.inf

        self.min_hist = deque(maxlen=learn_window_steps)
        self.max_hist = deque(maxlen=learn_window_steps)

        self.state = 0
        self.last_change_t = 0.0
        self.last_contact_tentative = None
        self.last_release_tentative = None

        self.mid = None
        self.lower = None
        self.upper = None

    def _update_levels_from_cycle(self):
        if self.curr_min < math.inf:
            self.min_hist.append(self.curr_min)
        if self.curr_max > -math.inf:
            self.max_hist.append(self.curr_max)

        if len(self.min_hist):
            new_contact = sorted(self.min_hist)[len(self.min_hist)//2]
            self.contact_level = (new_contact if self.contact_level is None
                                  else (1 - self.alpha) * self.contact_level + self.alpha * new_contact)
        if len(self.max_hist):
            new_noc = sorted(self.max_hist)[len(self.max_hist)//2]
            self.nocontact_level = (new_noc if self.nocontact_level is None
                                    else (1 - self.alpha) * self.nocontact_level + self.alpha * new_noc)

        # Decide orientation automatically once both levels exist
        if (self.contact_level is not None) and (self.nocontact_level is not None):
            self.invert = (self.contact_level > self.nocontact_level)

        self.curr_min = math.inf
        self.curr_max = -math.inf

    def _recompute_bands(self):
        if (self.contact_level is None) or (self.nocontact_level is None):
            self.mid = self.lower = self.upper = None
            return
        rng = max(1e-9, abs(self.nocontact_level - self.contact_level))

        # Auto-decide min band if not provided
        if self.clip_band_min is None:
            self.clip_band_min = 0.02 if rng < 50 else 5.0

        band = max(self.clip_band_min, min(self.clip_band_max, self.hfrac * rng))
        self.mid = 0.5 * (self.contact_level + self.nocontact_level)
        self.lower = self.mid - band
        self.upper = self.mid + band

    def update(self, v, t=None):
        if t is None:
            t = time.perf_counter()

        # Track extrema
        if v < self.curr_min:
            self.curr_min = v
        if v > self.curr_max:
            self.curr_max = v

        self._recompute_bands()
        dt_since_change = t - self.last_change_t

        # Bootstrap (wait until levels exist)
        if self.mid is None or self.invert is None:
            if dt_since_change > 0.25:
                self._update_levels_from_cycle()
                self._recompute_bands()
            return self.state, self.lower, self.mid, self.upper

        # --- Hysteresis + debounce + dwell ---
        if not self.invert:  # normal: contact < no-contact (FSR)
            if self.state == 0:  # no-contact
                if v >= self.lower and dt_since_change >= self.min_dwell:
                    if self.last_contact_tentative is None:
                        self.last_contact_tentative = t
                    elif (t - self.last_contact_tentative) >= self.min_contact:
                        self._update_levels_from_cycle()
                        self.state = 1
                        self.last_change_t = t
                        self.last_contact_tentative = None
                        self.curr_min = v
                        self.curr_max = v
                else:
                    self.last_contact_tentative = None
            else:  # contact
                if v <= self.upper and dt_since_change >= self.min_dwell:
                    if self.last_release_tentative is None:
                        self.last_release_tentative = t
                    elif (t - self.last_release_tentative) >= self.min_release:
                        self._update_levels_from_cycle()
                        self.state = 0
                        self.last_change_t = t
                        self.last_release_tentative = None
                        self.curr_min = v
                        self.curr_max = v
                else:
                    self.last_release_tentative = None

        else:  # inverted: contact > no-contact (Pressure sensor)
            if self.state == 0:  # no-contact
                if v <= self.upper and dt_since_change >= self.min_dwell:
                    if self.last_contact_tentative is None:
                        self.last_contact_tentative = t
                    elif (t - self.last_contact_tentative) >= self.min_contact:
                        self._update_levels_from_cycle()
                        self.state = 1
                        self.last_change_t = t
                        self.last_contact_tentative = None
                        self.curr_min = v
                        self.curr_max = v
                else:
                    self.last_contact_tentative = None
            else:  # contact
                if v >= self.lower and dt_since_change >= self.min_dwell:
                    if self.last_release_tentative is None:
                        self.last_release_tentative = t
                    elif (t - self.last_release_tentative) >= self.min_release:
                        self._update_levels_from_cycle()
                        self.state = 0
                        self.last_change_t = t
                        self.last_release_tentative = None
                        self.curr_min = v
                        self.curr_max = v
                else:
                    self.last_release_tentative = None

        return self.state, self.lower, self.mid, self.upper

class AdaptiveFSR_old:
    def __init__(self, init_contact=0.2, init_nocontact=1.0,
                 ema_alpha=0.2, hysteresis_frac=0.10,
                 min_contact_ms=60, min_release_ms=60, min_dwell_ms=120,
                 clip_band_min=0.02, clip_band_max=1e6, learn_window_steps=6):
        self.alpha = ema_alpha
        self.hfrac = hysteresis_frac
        self.min_contact = min_contact_ms / 1000.0
        self.min_release = min_release_ms / 1000.0
        self.min_dwell   = min_dwell_ms   / 1000.0
        self.clip_band_min = clip_band_min
        self.clip_band_max = clip_band_max

        self.contact_level   = init_contact
        self.nocontact_level = init_nocontact

        self.curr_min = math.inf
        self.curr_max = -math.inf

        self.min_hist = deque(maxlen=learn_window_steps)
        self.max_hist = deque(maxlen=learn_window_steps)

        self.state = 0
        self.last_change_t = 0.0
        self.last_contact_tentative = None
        self.last_release_tentative = None

        self.mid = None
        self.lower = None
        self.upper = None

# After each confirmed dwell, take the median of recent mins/maxes, then update contact_level 
# and nocontact_level with an EMA (so it adapts step-by-step but doesn’t jitter).
    def _update_levels_from_cycle(self):
        if self.curr_min < math.inf:
            self.min_hist.append(self.curr_min)
        if self.curr_max > -math.inf:
            self.max_hist.append(self.curr_max)

        if len(self.min_hist):
            new_contact = sorted(self.min_hist)[len(self.min_hist)//2]
            self.contact_level = (new_contact if self.contact_level is None
                                  else (1 - self.alpha) * self.contact_level + self.alpha * new_contact)
        if len(self.max_hist):
            new_noc = sorted(self.max_hist)[len(self.max_hist)//2]
            self.nocontact_level = (new_noc if self.nocontact_level is None
                                    else (1 - self.alpha) * self.nocontact_level + self.alpha * new_noc)

        self.curr_min = math.inf
        self.curr_max = -math.inf
# Compute the midpoint and hysteresis band (lower, upper) from the learned levels.
    def _recompute_bands(self):
        if (self.contact_level is None) or (self.nocontact_level is None):
            self.mid = self.lower = self.upper = None
            return
        rng = max(1e-9, abs(self.nocontact_level - self.contact_level))
        band = max(self.clip_band_min, min(self.clip_band_max, self.hfrac * rng))
        self.mid = 0.5 * (self.contact_level + self.nocontact_level)
        self.lower = self.mid - band
        self.upper = self.mid + band

    def update(self, v, t=None):
        """
        v: current raw sensor reading (FSR/pressure)
        t: timestamp in seconds (optional)
        """
        #print(v)
        if t is None:
            t = time.perf_counter()

        # track extrema in current dwell
        if v < self.curr_min:
            self.curr_min = v
        if v > self.curr_max:
            self.curr_max = v

        self._recompute_bands()
        dt_since_change = t - self.last_change_t

        # bootstrap
        if self.mid is None:
            if dt_since_change > 0.25:
                self._update_levels_from_cycle()
                self._recompute_bands()
            return self.state, self.lower, self.mid, self.upper

        # hysteresis + debounce + min dwell
        if self.state == 0:  # no-contact
            if v >= self.lower and dt_since_change >= self.min_dwell:
                if self.last_contact_tentative is None:
                    self.last_contact_tentative = t
                elif (t - self.last_contact_tentative) >= self.min_contact:
                    self._update_levels_from_cycle()
                    self.state = 1
                    self.last_change_t = t
                    self.last_contact_tentative = None
                    self.curr_min = v
                    self.curr_max = v
            else:
                self.last_contact_tentative = None
        else:  # contact
            if v <= self.upper and dt_since_change >= self.min_dwell:
                if self.last_release_tentative is None:
                    self.last_release_tentative = t
                elif (t - self.last_release_tentative) >= self.min_release:
                    self._update_levels_from_cycle()
                    self.state = 0
                    self.last_change_t = t
                    self.last_release_tentative = None
                    self.curr_min = v
                    self.curr_max = v
            else:
                self.last_release_tentative = None

        return self.state, self.lower, self.mid, self.upper


def low_pass_filter(new_value, last_filtered, alpha=0.1):
    return alpha * new_value + (1 - alpha) * last_filtered


def unwrap_heading(IMU_angle, last_angle, unwrapped_angle,
                   heading_anchor, last_state, time_now, pressure, Max_pressure,
                   swaylimit=12, stable_time=3.0,
                   stable_start=None, last_stable_angle=None):
    """
    Unwrap heading and determine gait state with robust stability detection.

    State machine:
      - Walking (1): within sway limit -> pressure = 0.4
      - Turning (0): outside sway -> pressure= 0
        -> Once heading stops drifting for >= stable_time, 
           set new anchor and return to walking.
    """

    # --- Unwrap IMU angle ---
    dtheta = IMU_angle - last_angle
    if dtheta > 180:
        dtheta -= 360
    elif dtheta < -180:
        dtheta += 360
    new_unwrapped = unwrapped_angle + dtheta

    state = last_state
    new_pressure = pressure
    new_anchor = heading_anchor
    new_time_now = perf_counter()
    new_stable_start = stable_start
    new_last_stable_angle = last_stable_angle

    # --- Logic ---
    if last_state == 1:  # walking
        if abs(new_unwrapped - heading_anchor) > swaylimit:
            # left sway zone -> turning
            state, new_pressure = 0, 0
            new_stable_start = None
            new_last_stable_angle = new_unwrapped

    elif last_state == 0:  # turning
        # Has heading stopped drifting?
        if last_stable_angle is None:
            new_last_stable_angle = new_unwrapped
            new_stable_start = perf_counter()
        else:
            if abs(new_unwrapped - last_stable_angle) < 5.0:  # <5° drift, before it was 1deg
                if new_stable_start is None:
                    new_stable_start = perf_counter()
                elif perf_counter() - new_stable_start >= stable_time:
                    # Stable long enough -> resume walking
                    state, new_pressure = 1, Max_pressure
                    new_anchor = new_unwrapped
                    new_stable_start = None
                    new_last_stable_angle = None
            else:
                # Still drifting -> reset timer
                new_last_stable_angle = new_unwrapped
                new_stable_start = None

    return (new_unwrapped, IMU_angle, state, new_pressure,
            new_anchor, state, new_time_now, new_stable_start, new_last_stable_angle)

## MAIN: Run the script #####################################################



def within_30_percent(actual, expected):
    # Access the .value for arithmetic
    lower_bound = expected.value * 0.7
    upper_bound = expected.value * 1.3
    return lower_bound <= actual.value <= upper_bound






if __name__ == "__main__":
    
    switch = mp.Value('i', 1)    # 0 ----FSR, else Pressure Sensor
    while True:
        try:
            choice = int(input("Select input mode (0 = FSR, 1 = Pressure Sensor): "))
            if choice in (0, 1):
                switch.value = choice
                print(f"Mode set: {'Pressure Sensor' if choice == 1 else 'FSR'}")
                break
            else:
                print("Invalid input. Please enter 0 or 1 only.")
        except ValueError:
            print("Invalid input. Please enter a number (0 or 1).")
    
    
    # mp. = multiple processinge
    ENABLE_LIVEPLOT = 1  # 0 will not plot live
    ENABLE_LIVEPLOT_cpg = 1 
    
    imu_on = 1              # 1 means IMU will be attempted to connect, 0 otherwise
    
    heelR = mp.Value('i', 0) # right heel strike (HS) live state
    heelL = mp.Value('i', 0)
    toeR  = mp.Value('i', 0) # right toe strike (TS) live state
    toeL  = mp.Value('i', 0)
    HPeriod_available_R = mp.Value('i', 0) # check for three heel strike
    HPeriod_available_L = mp.Value('i', 0)
    HeelRp = mp.Value('d', 1.4) # average period between HS
    HeelLp = mp.Value('d', 1.4)
    TPeriod_available_R = mp.Value('i', 0) # check for three toe strike
    TPeriod_available_L = mp.Value('i', 0)     
    ToeRp = mp.Value('d', 0) # average period between TS
    ToeLp = mp.Value('d', 0)
    
    Allexit = mp.Value('i', 0) 

    gaitR = mp.Value('d', 0) # 
    gaitL = mp.Value('d', 0)
    Rallow = mp.Value('i', 0) # used in the sub code
    Lallow = mp.Value('i', 0)
    supportTimeR     = mp.Value('d', 0)     # when actuator R starts 
    unsupportTimeR   = mp.Value('d', 0)   # when actuator R stops
    supportTimeL     = mp.Value('d', 0)
    unsupportTimeL   = mp.Value('d', 0)
    supportTimeRAn   = mp.Value('d', 0)
    unsupportTimeRAn = mp.Value('d', 0) 
    supportTimeLAn   = mp.Value('d', 0)
    unsupportTimeLAn = mp.Value('d', 0)

    record_start_fd = mp.Value('i', 0)
    r_pressed = mp.Value('i', 0) # for recording
    
    R = mp.Value('d', 0)        # Pressure output from Digital regulator of the 1st muscle right
    L = mp.Value('d', 0)
    RR = mp.Value('d', 0)       # Pressure output from Digital regulator of the 2nd muscle right
    LL= mp.Value('d', 0)       
    Emg_sig = mp.Value('d', 3)  # trigger EMG
    Applied_pressure= mp.Value('d', 0)
    Stage_of_user =  mp.Value('d', 2)

    Lr = mp.Value('d', 0)       # Loadcell value for right leg
    Ll = mp.Value('d', 0)
    psRu = mp.Value('d', 0)     # Pressure reading 1 
    psRd = mp.Value('d', 0)     # Pressure reading 2
    psLu = mp.Value('d', 0)
    psLd = mp.Value('d', 0)
    
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


    FSRrh = mp.Value('d', 0)    # FSR reading right heel (raw data)
    FSRrt = mp.Value('d', 0)
    FSRlh = mp.Value('d', 0)
    FSRlt = mp.Value('d', 0)
    c_time = mp.Value('d', 0)   # current time for DAQ


 
    rSitThigh = mp.Value('d', 0)    
    rSitShank = mp.Value('d', 0)
    lSitThigh = mp.Value('d', 0)
    lSitShank = mp.Value('d', 0)
    hipSit    = mp.Value('d', 0)   


    rStandThigh = mp.Value('d', 0)    
    rStandShank = mp.Value('d', 0)
    lStandThigh = mp.Value('d', 0)
    lStandShank = mp.Value('d', 0)
    hipStand    = mp.Value('d', 0)   
    

    rStrength = mp.Value('d', 0.4)    # starting
    rLength = mp.Value('d', 0.8)    
   
    lStrength = mp.Value('d', 0.4)   # starting  # 0.45, 0.7
    lLength = mp.Value('d', 0.8) #  0.1 0.4

    UserInfoGiven = mp.Value('d', 0)




    # ============================================================
    # CPG Shared Variables
    # ============================================================

    CPG_dt = Value('d', 0.0)

    CPG_phase_right = Value('d', 0.0)
    CPG_phase_left = Value('d', 0.0)

    CPG_phase_right_deg = Value('d', 0.0)
    CPG_phase_left_deg = Value('d', 0.0)

    CPG_goal_right = Value('d', 0.0)
    CPG_goal_left = Value('d', 0.0)

    CPG_goal_right_deg = Value('d', 0.0)
    CPG_goal_left_deg = Value('d', 0.0)

    CPG_lookup_right = Value('d', 0.0)
    CPG_lookup_left = Value('d', 0.0)

    CPG_simple_right = Value('d', 0.0)
    CPG_simple_left = Value('d', 0.0)

    CPG_error_right = Value('d', 0.0)
    CPG_error_left = Value('d', 0.0)

    CPG_error_right_deg = Value('d', 0.0)
    CPG_error_left_deg = Value('d', 0.0)

    CPG_theta_tr = Value('d', 0.0)
    CPG_theta_tl = Value('d', 0.0)

    CPG_theta_er = Value('d', 0.0)
    CPG_theta_el = Value('d', 0.0)

    CPG_y_tr = Value('d', 0.0)
    CPG_y_tl = Value('d', 0.0)

    CPG_y_er = Value('d', 0.0)
    CPG_y_el = Value('d', 0.0)

    CPG_torque_right = Value('d', 0.0)
    CPG_torque_left = Value('d', 0.0)

    CPG_pressure_right = Value('d', 0.0)
    CPG_pressure_left = Value('d', 0.0)
    CPG_actual_knee_right = mp.Value("d", 0.0)
    CPG_actual_knee_left = mp.Value("d", 0.0)




    timeforlog = time.strftime("%Y%m%d-%H%M%S")
    LogFile = "All"+"_"+"Data"+"_"+timeforlog+".csv"


 #rSitThigh, rSitShank, lSitThigh, lSitShank,   rStandThigh,  rStandShank, lStandThigh,  lStandShank,  hipStand,   rStrength,  lStrength, rLength, lLength



## Execute Multiple Process ############################################################
    Daq = mp.Process(target=record_daq, args=(Allexit, record_start_fd, c_time, LogFile, Lr, Ll, psRu, psRd, psLu,psLd, FSRrh, FSRrt, FSRlh, FSRlt,switch,heelR,heelL, HeelRp, HeelLp,toeR,toeL, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, IMU_11, IMU_22, IMU_33, IMU_44, IMU_55, IMU_1z, IMU_2z, IMU_3z, IMU_4z, IMU_5z,rSitThigh, rSitShank, lSitThigh, lSitShank,  hipSit, rStandThigh,  rStandShank, lStandThigh,  lStandShank,  hipStand,   rStrength,  lStrength, rLength, lLength, R, L, RR, LL, Emg_sig, CPG_dt, CPG_goal_right, CPG_goal_left, CPG_lookup_right, CPG_lookup_left,CPG_theta_tr, CPG_theta_tl,CPG_theta_er, CPG_theta_el,CPG_y_tr, CPG_y_tl,CPG_y_er, CPG_y_el, CPG_torque_right, CPG_torque_left   
  ))
    # record_daq = function to record daq while updating variables
    # args = variables
    if switch.value != 0:
        Foot_pressure = mp.Process(target=Pressure_foot, args=(Allexit,  FSRrh, FSRrt, FSRlh, FSRlt))

    FSR = mp.Process(target=Period_FSR, args=(heelR,heelL,toeR,toeL,\
        HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,\
        TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL))
    # Period_FSR = get period of FSR
    T_to_walk = mp.Process(target=Timesforwalk, args=(\
        HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,\
        TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL,\
        Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL,\
              supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn, rStrength, lStrength, rLength, lLength))
    # Timesforwalk = calculate gait period, actuation period 
    Activity_start = mp.Process(target=Activity, args=(heelR,heelL,toeR,toeL,\
        Allexit,\
        Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL,\
              supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn,\
                R, L, RR, LL, Emg_sig, r_pressed, IMU_11, IMU_22,IMU_33,IMU_44, IMU_5z,IMU_55, Applied_pressure,Stage_of_user, rSitThigh, rSitShank, lSitThigh, lSitShank, rStandThigh, rStandShank, lStandThigh, lStandShank, psRu, psRd, psLu, psLd, UserInfoGiven,HeelRp, HeelLp, CPG_dt, CPG_goal_right, CPG_goal_left, CPG_lookup_right, CPG_lookup_left,CPG_theta_tr, CPG_theta_tl,CPG_theta_er, CPG_theta_el,CPG_y_tr, CPG_y_tl,CPG_y_er, CPG_y_el, CPG_torque_right, CPG_torque_left, CPG_actual_knee_right, CPG_actual_knee_left , "right", ))
    # Walk = execute the actuation for walkings
   
    FSR.start() 
    print("FSR PID:", FSR.pid, "alive:", FSR.is_alive(), flush=True )
    
    T_to_walk.start()
    print("T_to_walk PID:", T_to_walk.pid, "alive:", T_to_walk.is_alive(), flush=True )
    
    Activity_start.start()
    print("Activity start PID:", Activity_start.pid, "alive:", Activity_start.is_alive(), flush=True )

    Daq.start()   
    print("DAQ read start:", Daq.pid, "alive:", Daq.is_alive(), flush=True )

    if switch.value != 0:
         Foot_pressure.start()
         
         print("Foot Pressure Reading active:", Foot_pressure.pid, "alive:", Foot_pressure.is_alive(), flush=True )

    # Library for IMUs
    
    global quit
    global record_interval

    global IMU_1, IMU_2, IMU_3, IMU_4, IMU_5, IMU_5zz
    IMU_1, IMU_2, IMU_3, IMU_4, IMU_5, IMU_5zz = 0,0,0,0,0,0

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
    
    firsttime = 1
    record_interval = 0  
   
    if switch.value == 0: # FSR
        det_rh = AdaptiveFSR_old()  # Right Heel
        det_rt = AdaptiveFSR_old()  # Right Toe
        det_lh = AdaptiveFSR_old()  # Left Heel
        det_lt = AdaptiveFSR_old()  # Left Toe
    
    
    else: # pressure
        det_rh = AdaptiveFSR()  # Right Heel
        det_rt = AdaptiveFSR()  # Right Toe
        det_lh = AdaptiveFSR()  # Left Heel
        det_lt = AdaptiveFSR()  # Left Toe
         

    shared = {
    "Lr": Lr, "Ll": Ll,
    "R": R, "L": L, "RR": RR, "LL": LL,
    "psRu": psRu, "psRd": psRd, "psLu": psLu, "psLd": psLd,
    "FSRrh": FSRrh, "FSRrt": FSRrt, "FSRlh": FSRlh, "FSRlt": FSRlt,
    "heelR": heelR, "heelL": heelL, "toeR": toeR, "toeL": toeL,
    "HeelRp": HeelRp, "HeelLp": HeelLp,
    "supportTimeR": supportTimeR, "unsupportTimeR": unsupportTimeR,
    "supportTimeL": supportTimeL, "unsupportTimeL": unsupportTimeL,
    "IMU_11": IMU_11, "IMU_22": IMU_22, "IMU_33": IMU_33, "IMU_44": IMU_44, "IMU_55": IMU_55, "IMU_5z": IMU_5z 
    }
    
    shared_cpg = {
    "CPG_dt": CPG_dt,

    "CPG_goal_right": CPG_goal_right,
    "CPG_goal_left": CPG_goal_left,

    "CPG_theta_tr": CPG_theta_tr,
    "CPG_theta_tl": CPG_theta_tl,

    "CPG_actual_knee_right": CPG_actual_knee_right,
    "CPG_actual_knee_left": CPG_actual_knee_left,

    "CPG_torque_right": CPG_torque_right,
    "CPG_torque_left": CPG_torque_left,

    "CPG_pressure_right": R,
    "CPG_pressure_left": L,
}
    if ENABLE_LIVEPLOT:
        from liveplot import live_plot
        plotter = mp.Process(target=live_plot, args=(shared, Allexit))
        plotter.start()

    if ENABLE_LIVEPLOT_cpg:
        from liveplot_cpg import live_plot_cpg

        plotter_cpg = mp.Process(target=live_plot_cpg, args=(shared_cpg, Allexit))

        plotter_cpg.start()
    # try = connect to all IMUs
    openzen.set_log_level(openzen.ZenLogLevel.Warning)
        # Setting Arduino (run once)
    # lab COM3, home COM15, portenta COM53
    arduino = serial.Serial('COM53', 115200, parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS, timeout=.1) #115200
    # command to arduino to execute the EMG and actuators
    z = '<3, 0.0, 0.0, 0.0, 0.0, 0.0>'
    arduino.write(bytes(z, "utf-8"))
    time.sleep(2)

        ## Internal Function: set applied pressure
    def apply_pressure(e, r, l, rm, lm): # e = EMG, r = right actuator 1, rm = right actuator 2, ...
        z = '<' + str(e) +',' + str(r) +',' +  str(l) +',' + str(rm) +',' + str(lm) + '>'
        #print(z)
        arduino.write(bytes(z, "utf-8"))

 

    try:      
        if (imu_on):
          (client, sensor1, imu1, sensor2, imu2, sensor3, imu3, sensor4, imu4, sensor5, imu5) = connectSensors()
          quit = False

        # If the connecton completes, ...
          data_thread = threading.Thread(target=data_acquisition, args=(client, imu1, imu2, imu3, imu4, imu5, IMU_11,IMU_22,IMU_33, IMU_44, IMU_55, IMU_1z, IMU_2z,IMU_3z,IMU_4z,IMU_5z))
          data_thread.start()
        file_record()       # only create new files if connections are successful
        c_time.value = 0 # current time
        
        #apply_pressure(0, 0.1, 0.1, 0.1, 0.1)    
        mlocal = 0
        r_pressed.value = 0
        print("\nPress R to start recording or Q to Stop or U to provide participant info or E to exit")
        print("\nZ for sit to stand support...")
        print("\nW for walk support...")
        print("\nO for stand to sit support...")
        print("\nU to provide IMU data...")
        print("\nC for provide time coefficients...")

        print("\n Right Heel - Toe - Left Heel - Toe - Right Thigh - Shank - Left Thigh - Shank - HipR - Hiplean - State - Pressure  - EMG")
        
        last_angle = 0
        unwrapped = 0
        heading_anchor = 0
        last_state = 1
        time_now = perf_counter()
        
        
        Max_pressure = 0.6 # max pressure during standing
        pressure = Max_pressure
        stable_start = None
        last_stable_angle = None
        heading_anchor = copy.copy(IMU_5z.value)

        user_info_given = 0
        gait_state = "Sitting"
        vv = 2
        Stage_of_user.value = vv

        c1 = 0
        c2 = 0
        c3 = 0
        c4 = 0

        walk_allow = 0
        time_initi = perf_counter() 
        #heelR.value = 1
       # heelL.value = 0
        off = 0
        while True:
            """
            raw_yaw = copy.copy(IMU_5z.value)  # from shared variable

            (unwrapped, last_angle, state, pressure,
             heading_anchor, last_state, time_now,
                stable_start, last_stable_angle) = unwrap_heading(
                   raw_yaw, last_angle, unwrapped,
                      heading_anchor, last_state, time_now, pressure, Max_pressure,
                             swaylimit=30, stable_time=.8,
                                     stable_start=stable_start,
                                         last_stable_angle=last_stable_angle
                )
            Applied_pressure.value = copy.copy(pressure)
            """

           # print(f"Yaw={unwrapped:4.1f}, heading={heading_anchor:4.2f}, State={state}, Pressure={pressure:.2f}", end ='\r')
            #  print(f"State={state}, Pressure={pressure}, err={err_dbg:.1f} Yaw={raw_yaw:.2f}")

            # for recording ########################################################################################
            
     
            # IMU functions for detecting turning... can be placed here
            
            """
            time_now = perf_counter() 
            if   time_now -time_initi >= 0.7:     
                heelR.value = 1 - heelR.value
                heelL.value = 1 - heelL.value
            

                time_initi = time_now

                
            """    
            """
            time_now = perf_counter()

            heelR.value = 0
            heelL.value = 0

            if time_now >= time_initi:

                elapsed = time_now - time_initi

                if elapsed < pulse_width:
                    heelR.value = 1

                elif half_gait <= elapsed < half_gait + pulse_width:
                    heelL.value = 1

                if elapsed >= gait_time:
                    time_initi += gait_time
            """
            
              
            
            
            if keyboard.is_pressed('r'):                
                r_pressed.value = 1
                Emg_sig.value = 0
                if firsttime == 1:
                    
                    c_time.value = perf_counter() # record the current time from perf_counter()
                    record_interval = copy.copy(c_time.value)
                    firsttime = 0
                    print("\n\nRecording started -- Press Q to Stop, W to walk Support and K to Stop support")
                    print("Right Heel - Toe - Left Heel - Toe - Right Thigh - Shank - Left Thigh - Shank - HipR - Hiplean - State - Pressure")
                record_start  = 1
                record_start_fd.value = 1
            # print("here")
            

            ## Calculate Threshold #################################################################################
            
            if True:#switch.value == 0:  ##############   FSR
                
               # if vv==1 or vv== 1.5 or vv==4:
               # now = time.perf_counter()
                
                #    state_RH, lo, mid, hi = det_rh.update(0)  # Right heel
                    
                 #   state_RT, _,  _,  _  = det_rt.update(0)  # Right toe
                    
                 #   state_LH, _,  _,  _  = det_lh.update(0)  # Left heel
                    
                #    state_LT, _,  _,  _  = det_lt.update(0)  # Left to
                #else:
                state_RH, lo, mid, hi = det_rh.update(FSRrh.value)  # Right heel
                
                state_RT, _,  _,  _  = det_rt.update(FSRrt.value)  # Right toe
                
                state_LH, _,  _,  _  = det_lh.update(FSRlh.value)  # Left heel
                
                state_LT, _,  _,  _  = det_lt.update(FSRlt.value)  # Left to
                    
                
                
                #"""
                if switch.value == 0:
                    heelR.value = 1- state_RH # inverting the state as heel strike produce 0 volt
                    toeR.value = 1- state_RT
                    heelL.value = 1- state_LH
                    toeL.value = 1- state_LT
                #"""
                else:  # pressure
                #if True:
                    heelR.value = state_RH # inverting the state as heel strike produce 0 volt
                    toeR.value = state_RT
                    heelL.value = state_LH
                    toeL.value = state_LT
                                     
           
               
           # print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end ='\r' ) 
          #  print('     {:f},        {:f},       {:f},       {:f},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}'.format((FSRrh.value),  (FSRrt.value), (FSRlh.value),  (FSRlt.value),float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end ='\r' ) 
          #  print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end ='\r' ) 
           # print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_1z.value),  float(IMU_2z.value), float(IMU_3z.value),  float(IMU_4z.value), float(IMU_5z.value), float(pressure)), end ='\r' ) 
           # print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}, {:.3f}, {}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_11.value),  float(IMU_22.value), float(IMU_33.value),  float(IMU_44.value), float(IMU_5z.value),  float(IMU_55.value), {gait_state}, float(pressure)), end ='\r' ) 
           # print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}, {:.3f}, {:1f}, {:.3f}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_11.value),  float(IMU_22.value), float(IMU_33.value),  float(IMU_44.value), float(IMU_5z.value),  float(IMU_55.value), float(vv), float(pressure), float(Emg_sig.value)), end ='\r' ) 
            print('          {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}, {:.3f}, {:1f}, {:.3f}, {:.3f}'.format(float(IMU_11.value),  float(IMU_22.value), float(IMU_33.value),  float(IMU_44.value), float(IMU_5z.value),  float(IMU_55.value), float(vv), float(pressure), float(Emg_sig.value)), end ='\r' ) 
        
         
         #   print('     {:.3f}, {:.3f}'.format( float(IMU_5z.value), float(THETA)), end ='\r' ) 
          #  print('            
            if keyboard.is_pressed('q') and r_pressed.value==1:
                        #taskAI.stop()
                        Emg_sig.value = 3
                        record_start  = 0
                        record_start_fd.value = 0
                        firsttime = 1
                
                        s1, s2, s3, s4, s5 = 1,1,1,1,1

               
                        print("\n Recording stopped")
                        r_pressed.value = 0
                                
                        # filing
                        mlocal = 0
                        print("\nPress R to record another trial, U to update participant info or E to Exit")
                        print("Right Heel - Toe - Left Heel - Toe - Right Thigh - Shank - Left Thigh - Shank - HipR - Hiplean - State - Pressure")
            
                        #while True:
                        # Analogs = taskAI.read()

            #if keyboard.is_pressed('z') :
             #    Emg_sig.value = 0
              #   print("EMG off, press R to on again")

            ## Exit the whole program ###################################################################            
            if keyboard.is_pressed('e') and r_pressed.value == 0:
                                quit = True
                                Allexit.value = 1 # to quit all mp.process
                                mlocal = 1
                                break
            
            if keyboard.is_pressed('w') and walk_allow == 0:
                     walk_allow = 1
                     print("\n walk support activated")
                     off = 0
            if keyboard.is_pressed('k') and walk_allow == 1:
                walk_allow = 0
                print("\n No walk support")
            
            if walk_allow == 1 and r_pressed.value==1:
                #  print("hello")
                  apply_pressure(Emg_sig.value , R.value, L.value, RR.value, LL.value)
            elif off == 0:
                            R.value = 0
                            RR.value=  0
                            L.value = 0
                            LL.value= 0
                            apply_pressure(Emg_sig.value , R.value, L.value, RR.value, LL.value)
                            off = 1

    ## If IMU connection fails, it will come here               
    except KeyboardInterrupt:
        pass
    ## If IMU connection succeeds and the program runs through, 
    finally:
        
        Emg_sig.value = 3
        R.value = 0
        RR.value=  0
        L.value = 0
        LL.value= 0
        apply_pressure(Emg_sig.value , 0, 0, 0, 0)
        arduino.close()
            
        quit = True
        Allexit.value = 1 # to quit all mp.process
        if (imu_on):
            data_thread.join()
            print ("\nStreaming of sensor data complete")
            client.close()    
        f1.close()
        f2.close()
        f3.close()
        f4.close()
        f5.close()

        # To exit mp.process
        # To exit mp.process safely
        processes = [FSR, T_to_walk, Activity_start, Daq]
       # processes = [ Activity_start, Daq]

        if ENABLE_LIVEPLOT:
            processes.append(plotter)
        if ENABLE_LIVEPLOT_cpg:
            processes.append(plotter_cpg)

        
        if switch.value != 0:
            processes.append(Foot_pressure)

        for p in processes:
            p.join(timeout=3)

        for p in processes:
            if p.is_alive():
                print(f"Process {p.name} did not stop. Terminating...")
                p.terminate()
                p.join(timeout=2)
                
        #old 
        #FSR.join()
        #T_to_walk.join()
        #Activity_start.join()
        #Daq.join()
        #if ENABLE_LIVEPLOT:
        #    plotter.join()
        
        #if switch.value != 0:
         #    Foot_pressure.join()
   # rd.join()


## SAVE FILES ###########################################################################
#########################################################################################

        a = pandas.read_csv(LogFile)
        b = pandas.read_csv(LogFile1)
        c = pandas.read_csv(LogFile2)
        d = pandas.read_csv(LogFile3)
        e = pandas.read_csv(LogFile4)
        f = pandas.read_csv(LogFile5)
        
        output4 = pandas.concat((a,b,c,d,e,f), axis='columns')
        
        output4 = output4.loc[:, ~output4.columns.str.contains("Unnamed")]        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        file_name = f"Alldata_{timestamp}.csv"
            
        output4.to_csv(file_name, index=False)                 
        
        df = pd.read_csv(LogFile)

    # Compute differences to detect time reset
        time_diff = df["Time"].diff()
        reset_indices = df.index[time_diff < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indices = [0] + reset_indices + [len(df)]

        # Create output folder
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indices) - 1):
            start_idx = reset_indices[i]
            end_idx = reset_indices[i + 1]
            trial_df = df.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"daq_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
            
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
        
        dfrt = pd.read_csv(LogFile1)

    # Compute differences to detect time reset
        time_diffrt = dfrt["rtTime"].diff()
        reset_indicesrt = dfrt.index[time_diffrt < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indicesrt = [0] + reset_indicesrt + [len(dfrt)]

        # Create output folder
    # output_dir = "split_trials"
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indicesrt) - 1):
            start_idx = reset_indicesrt[i]
            end_idx = reset_indicesrt[i + 1]
            trial_df = dfrt.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"rt_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
        
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")

################################

        dfrs = pd.read_csv(LogFile2)

    # Compute differences to detect time reset
        time_diffrs = dfrs["rsTime"].diff()
        reset_indicesrs = dfrs.index[time_diffrs < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indicesrs = [0] + reset_indicesrs + [len(dfrs)]

        # Create output folder
        #output_dir = "split_trials"
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indicesrs) - 1):
            start_idx = reset_indicesrs[i]
            end_idx = reset_indicesrs[i + 1]
            trial_df = dfrs.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"rs_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
        
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
                        
###################################
        dflt = pd.read_csv(LogFile3)

    # Compute differences to detect time reset
        time_difflt = dflt["ltTime"].diff()
        reset_indiceslt = dflt.index[time_difflt < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indiceslt = [0] + reset_indiceslt + [len(dflt)]

        # Create output folder
        #output_dir = "split_trials"
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indiceslt) - 1):
            start_idx = reset_indiceslt[i]
            end_idx = reset_indiceslt[i + 1]
            trial_df = dflt.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"lt_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
            
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
                        
####################################################

        dfls = pd.read_csv(LogFile4)

    # Compute differences to detect time reset
        time_diffls = dfls["lsTime"].diff()
        reset_indicesls = dfls.index[time_diffls < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indicesls = [0] + reset_indicesls + [len(dfls)]

        # Create output folder
        #output_dir = "split_trials"
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indicesls) - 1):
            start_idx = reset_indicesls[i]
            end_idx = reset_indicesls[i + 1]
            trial_df = dfls.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"ls_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
        
            trial_df.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
                        
################################################################

        dfh = pd.read_csv(LogFile5)

    # Compute differences to detect time reset
        time_diffh = dfh["hTime"].diff()
        reset_indicesh = dfh.index[time_diffh < -0.1].tolist()

        # Include start and end of DataFrame
        reset_indicesh = [0] + reset_indicesh + [len(dfh)]

        # Create output folder
        #output_dir = "split_trials"
        output_dir = f"Experiment_{timestamp}"
        os.makedirs(output_dir, exist_ok=True)

        # Split and save each trial
        for i in range(len(reset_indicesh) - 1):
            start_idx = reset_indicesh[i]
            end_idx = reset_indicesh[i + 1]
            trial_df = dfh.iloc[start_idx:end_idx].reset_index(drop=True)
            
            # Use current timestamp or from data if available
            
            file_name = f"h_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            
            trial_df = trial_df.loc[:, ~trial_df.columns.str.contains("Unnamed")]
        
            trial_df.to_csv(file_path, index=False)
            
            print(f"Saved: {file_path}")
    ###################################################################
    # 
    # 
    #                        
    # cwd = os.getcwd() 
        #split_trials_dir = os.path.join(cwd, "..", "split_trials") 
        #input_dir = r"C:\Users\jf20027\Desktop\LocalDir\split_trials"
        #data_files = '../split_trials/'
        #csv_name = 'data.csv'
        
        base_dir = r"C:\Users\jf20027\Desktop\LocalDir"
        input_dir = os.path.join(base_dir, f"Experiment_{timestamp}")

        
        for i in range(len(reset_indices) - 1):
            
            #files = os.listdir(cwd)
            #print(cwd)
            #pd.read_csv(f"{data_files}{csv_name}")
            daq_file = os.path.join(input_dir, f"daq_{i+1}_{timestamp}.csv")
            rt_file  = os.path.join(input_dir, f"rt_{i+1}_{timestamp}.csv")
            rs_file  = os.path.join(input_dir, f"rs_{i+1}_{timestamp}.csv")
            lt_file  = os.path.join(input_dir, f"lt_{i+1}_{timestamp}.csv")
            ls_file  = os.path.join(input_dir, f"ls_{i+1}_{timestamp}.csv")
            h_file   = os.path.join(input_dir, f"h_{i+1}_{timestamp}.csv")
            
            daq_df = pd.read_csv(daq_file)
            rt_df = pd.read_csv(rt_file)
            rs_df = pd.read_csv(rs_file)
            lt_df = pd.read_csv(lt_file)
            ls_df = pd.read_csv(ls_file)
            h_df  = pd.read_csv(h_file)
            
            #file_name_daq= pandas.read_csv(f"daq_{i+1}_{timestamp}.csv")
            #file_name_rt = pandas.read_csv(f"rt_{i+1}_{timestamp}.csv")
            #file_name_rs = pandas.read_csv(f"rs_{i+1}_{timestamp}.csv")
            #file_name_lt = pandas.read_csv(f"lt_{i+1}_{timestamp}.csv")
            #file_name_ls = pandas.read_csv(f"ls_{i+1}_{timestamp}.csv")
            #file_name_h  = pandas.read_csv(f"h_{i+1}_{timestamp}.csv")
            output4 = pandas.concat((daq_df,rt_df,rs_df,lt_df,ls_df,h_df), axis='columns')
        
            output4 = output4.loc[:, ~output4.columns.str.contains("Unnamed")]        
            #output4.to_csv(f"trial_{i+1}_{timestamp}.csv", index=False)                 
        
            file_name = f"trial_{i+1}_{timestamp}.csv"
            file_path = os.path.join(output_dir, file_name)
            
            output4.to_csv(file_path, index=False)
            print(f"Saved: {file_path}")
        
        print("deleting unnecessary files...this might takes a while")
        LogFile1 = os.path.join(base_dir, f"LP_1_{timestr}.csv")
        if(os.path.exists(LogFile1) and os.path.isfile(LogFile1)):
                            os.remove(LogFile1)
        LogFile2 = os.path.join(base_dir, f"LP_2_{timestr}.csv")    
        if(os.path.exists(LogFile2) and os.path.isfile(LogFile2)):
                            os.remove(LogFile2)
                            #print("files deleted")
        LogFile3 = os.path.join(base_dir, f"LP_3_{timestr}.csv")                    
        if(os.path.exists(LogFile3) and os.path.isfile(LogFile3)):
                            os.remove(LogFile3)
        LogFile4 = os.path.join(base_dir, f"LP_4_{timestr}.csv")
        if(os.path.exists(LogFile4) and os.path.isfile(LogFile4)):
                            os.remove(LogFile4)
        
        LogFile5 = os.path.join(base_dir, f"LP_5_{timestr}.csv")                
        if(os.path.exists(LogFile5) and os.path.isfile(LogFile5)):
                            os.remove(LogFile5)
        LogFile = os.path.join(base_dir, f"All_Data_{timeforlog}.csv")
        if(os.path.exists(LogFile) and os.path.isfile(LogFile)):
                            os.remove(LogFile)
        

   # main()
    