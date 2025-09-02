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
    data = ["rtTime"]  + ["R Thigh Angle"] 
    for ele in data:
                        f1.write(str(ele)+',')
    f1.write('\n')
 
    LogFile2 = "LP_2"+"_"+timestr+".csv"   # filename of output data file
    f2=open(LogFile2,'w')
    data = ["rsTime"]  + ["R Shank Angle"] 
    for ele in data:
                        f2.write(str(ele)+',')
    f2.write('\n')
 
    LogFile3 = "LP_3"+"_"+timestr+".csv"   # filename of output data file
    f3=open(LogFile3,'w')
    data = ["ltTime"]  + ["L Thigh Angle"] 
    for ele in data:
                        f3.write(str(ele)+',')
    f3.write('\n')
 
    LogFile4 = "LP_4"+"_"+timestr+".csv"   # filename of output data file
    f4=open(LogFile4,'w')
    data = ["lsTime"]  + ["L Shank Angle"] 
    for ele in data:
                        f4.write(str(ele)+',')
    f4.write('\n')
 
    LogFile5 = "LP_5"+"_"+timestr+".csv"   # filename of output data file
    f5=open(LogFile5,'w')
    data = ["hTime"]  + ["Hip Angle"] 
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
    error, sensor1 = client.obtain_sensor_by_name("Bluetooth", sensor1MacID)
    if not error == openzen.ZenError.NoError:
        print ("Error connecting to right thigh", sensor1MacID)
        quit = True
        sys.exit(1)
    imu1 = sensor1.get_any_component_of_type(openzen.component_type_imu)

    # Connect to sensor2
    error, sensor2 = client.obtain_sensor_by_name("Bluetooth", sensor2MacID)
    if not error == openzen.ZenError.NoError:
        print ("Error connecting to right shank", sensor2MacID)
        quit = True
        sys.exit(1)
    imu2 = sensor2.get_any_component_of_type(openzen.component_type_imu)

    # Connect to sensor3
    error, sensor3 = client.obtain_sensor_by_name("Bluetooth", sensor3MacID)
    if not error == openzen.ZenError.NoError:
        print ("Error connecting to left thigh", sensor3MacID)
        quit = True
        sys.exit(1)
    imu3 = sensor3.get_any_component_of_type(openzen.component_type_imu)

    error, sensor4 = client.obtain_sensor_by_name("Bluetooth", sensor4MacID)
    if not error == openzen.ZenError.NoError:
        print ("Error connecting to left shank", sensor4MacID)
        quit = True
        sys.exit(1)
    imu4 = sensor4.get_any_component_of_type(openzen.component_type_imu)
    
    error, sensor5 = client.obtain_sensor_by_name("Bluetooth", sensor5MacID)
    if not error == openzen.ZenError.NoError:
        print ("Error connecting to hip", sensor5MacID)
        quit = True
        sys.exit(1)
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

## Function: acquire IMUs (from Supplier's library) #####################################################
def data_acquisition(client, imu1, imu2, imu3, imu4, imu5):
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
            data = copy.copy(imu_data.r[0])
            IMU_1 = data
            rt = data
            ts = copy.copy(imu_data.timestamp)
            if (s1 ==1 and record_start==1):
                  s1 = 0
                  t1now = imu_data.timestamp
            
           # data1 = [ts - t1now] + [perf_counter()- detected_time] + [data] 
            data1 = [ts - t1now]  + [data] 
            
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
            rs = data
            ts = copy.copy(imu_data.timestamp)
            if (s2 ==1 and record_start==1):
                  s2 = 0
                  t2now = imu_data.timestamp
            #data2 = [ts - t2now] + [perf_counter()- detected_time] + [data] 
            data2 = [ts - t2now]  + [data] 
      
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
            ts = copy.copy(imu_data.timestamp)
            if (s3 ==1 and record_start==1):
                  s3 = 0
                  t3now = imu_data.timestamp
           # data3 = [ts - t3now] + [perf_counter()- detected_time] + [data] 
            data3 = [ts - t3now]  + [data] 

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
            ts = copy.copy(imu_data.timestamp)
            if (s4 ==1 and record_start==1):
                  s4 = 0
                  t4now = imu_data.timestamp
            #data4 = [ts - t4now] + [perf_counter()- detected_time] + [data] 
            data4 = [ts - t4now]  + [data] 
      
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
            ts = copy.copy(imu_data.timestamp)
            if (s5 ==1 and record_start==1):
                  s5 = 0
                  t5now = imu_data.timestamp
           # data5 = [ts - t5now] + [perf_counter()- detected_time] + [data] 
            data5 = [ts - t5now]  + [data] 

            if record_start == 1:
                for ele in data5:
                        f5.write(str(ele)+',')
                f5.write('\n')
 
        time.sleep(0.001)

    print("Data data_acquisition thread terminated")

## Function: calculate FSR period ####################################################
def Period_FSR(heelR,heelL,toeR,toeL, HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL):

     # Right heel
     heelcountR = 0         # Right heel strike count
     heelcountR_flag = 0    # Flag to enable state change
     heelcountTime_R =0     # exact time of strike
     Previous_heelcountTime_R = 0   
     timeHeelR = 0          # period between strike
     timeHeelR_avg = 0      # average of period between last three strikes
     # Left heel
     heelcountL, heelcountL_flag, heelcountTime_L, Previous_heelcountTime_L, timeHeelL, timeHeelL_avg = 0,0,0,0,0,0
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
         if HeelStateR == 1 and heelcountR_flag == 0:
             heelcountR_flag = 1
             heelcountTime_R = perf_counter()
             if heelcountR == 0:
                  gaitR.value = heelcountTime_R
             forToeR = copy.copy(heelcountTime_R) # to use in the toe off time calculation
             heelcountR = heelcountR +1  
            # Prev_heelcountTime_R = perf_counter()
             #print("here")   
             if heelcountR> 1:
                  time_bet_heel_R = heelcountTime_R - Previous_heelcountTime_R 
                  timeHeelR = timeHeelR + time_bet_heel_R 
             if heelcountR == 3:
                 timeHeelR_avg = timeHeelR/3 
                 timeHeelR = 0
                 heelcountR = 0 
                 gaitR.value =  perf_counter()
             Previous_heelcountTime_R = heelcountTime_R
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
                  timeHeelL = timeHeelL + time_bet_heel_L 
             if heelcountL == 3:
                 timeHeelL_avg = timeHeelL/3 
                 timeHeelL = 0
                 heelcountL = 0   
             Previous_heelcountTime_L = heelcountTime_L
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
              
     except Exception as e:
            import traceback
            print("FSR crashed", e , flush=True )
            traceback.print_exc()                  

## Function: calculates Actuation time based on Period #######################################
def Timesforwalk(HPeriod_available_R, HPeriod_available_L,HeelRp, HeelLp,Allexit,TPeriod_available_R, TPeriod_available_L,ToeRp, ToeLp,gaitR, gaitL, Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn):
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
                        supportTimeR.value= (0)*Rhtime #((Rttime/Rhtime) - 0.2 +r 0.2)*Rhtime # Toe off start 
                        unsupportTimeR.value= (0.45)*Rhtime#((Rttime/Rhtime) - 0.1)*Rhtime #90% of total period #((Rttrime/Rhtime) + 0.1)*Rhtime
                        #back 10-45
                        supportTimeRAn.value= (0)*Rhtime #0% of total period#((Rttime/Rhtime) - 0.15)*Rhtime
                        unsupportTimeRAn.value= (0.3)*Rhtime#((Rttime/Rhtime) - 0.25)*Rhtime # Toe off 
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
                            supportTimeL.value= (0)*Lhtime#((Lttime/Lhtime) - 0.2 + 0.2)*Lhtime
                            unsupportTimeL.value= (0.45)*Lhtime#0.6*Lhtime #((Lttime/Lhtime) + 0.1)*Lhtime
                            supportTimeLAn.value= (0)*Lhtime #((Lttime/Lhtime) - 0.15)*Lhrtime
                            unsupportTimeLAn.value= (0.3)*Lhtime #((Lttime/Lhtime) + 0.2)*Lhtime
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
def Walk(heelR,heelL,toeR,toeL,Allexit, Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL, supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn, R, L, RR, LL, Emg_sig, r_pressed):
    # Setting Arduino (run once)
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
 
    while not Allexit.value :
            # Get time when right heel strike
            if heelR.value==1:
                    gaittimeR= perf_counter() 
            # Get time when left heel strike          
            if heelL.value==1:
                   gaittimeL= perf_counter() 

            if Rallow.value ==1: # waiting for first three steps
                currenttime = perf_counter() - gaittimeR
                #print(currenttime,supportTimeR, unsupportTimeR)
                if currenttime>=supportTimeR.value and currenttime<=unsupportTimeR.value:                                    
                    R.value = 0.3
                else:    
                    R.value = 0

                #print(currenttime, supportTimeR,unsupportTimeR, supportTimeRAn,unsupportTimeRAn  )
                if currenttime>=supportTimeRAn.value and currenttime<=unsupportTimeRAn.value:
                    RR.value = 0.3
                else:    
                    RR.value = 0
                    
            if Lallow.value ==1:        
                currenttimel = perf_counter() - gaittimeL
                if currenttimel>=supportTimeR.value and currenttimel<=unsupportTimeR.value:
                    L.value = 0.3
                else:    
                    L.value = 0
                
                # if currenttimel>=supportTimeLAn and currenttimel<=unsupportTimeLAn:
                if currenttimel>=supportTimeRAn.value and currenttimel<=unsupportTimeRAn.value:                                        
                    LL.value = 0.3
                else:    
                    LL.value = 0
        
            # start actuation when record 
            if r_pressed.value == 1:
                apply_pressure(Emg_sig.value, R.value,L.value,RR.value,LL.value)
            else:
                apply_pressure(0,0,0,0,0) 

    apply_pressure(0,0,0,0,0)

## Function: start recording DAQ ###################################################
def record_daq(Allexit, record_start_fd, c_time, LogFile, Lr, Ll, psRu, psRd, psLu,psLd, FSRrh, FSRrt, FSRlh, FSRlt, switch ):      # varaibles need to change
    
    ## Convert funtion: Pressure sensors ###########################################
    def Volt_to_kPa(v):
         kPa = 73.3102*v - 36.7266
         return kPa
    ## Convert funtion: Load cell ##################################################
    def Volt_to_newton(v):
         Newton = 57.8137*v - 19.9659
         return Newton
    
    # DAQ initialisation (run once)
    taskAI = nidaqmx.Task()
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai0", min_val=0, max_val= 5)    # Loadcell right
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai1", min_val=0, max_val= 5)    # Loadcell left
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai2", min_val=0, max_val= 5)    # Pressure sensor right 
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai3", min_val=0, max_val= 5)    # Pressure sensor left
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai4", min_val=0, max_val= 5)    # FSR right heel
    
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai5", min_val=0, max_val= 5)    # FSR right toe
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai6", min_val=0, max_val= 5)    # FSR left heel
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai7", min_val=0, max_val= 5)    # FSR left toe
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai16", min_val=0, max_val= 5)   # FSR left heel
    taskAI.ai_channels.add_ai_voltage_chan("Dev5/ai17", min_val=0, max_val= 5)   # FSR left toe
    taskAI.timing.cfg_samp_clk_timing(1000, source="OnboardClock", active_edge=Edge.RISING, sample_mode=AcquisitionType.CONTINUOUS,samps_per_chan= 2)  # samps_per_chan= 100
            #10 works, 13 works
    taskAI.in_stream.input_buf_size = 120000

    # Start DAQ
    taskAI.start()   
    fa=open(LogFile,'w')
    data = ["Time"]  + ["R Loadcell"] + ["L Loadcell"]  + ["Pressure R BAM UP"] + ["Pressure R BAM Down"] + ["Pressure L BAM UP"] + ["Pressure L BAM Down"] + ["FSR R Heel"] + ["FSR R Toe"] + ["FSR L Heel"] + ["FSR L Toe"]
            
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
        psLu.value         = Volt_to_kPa(valuex[3])
        psRd.value         = Volt_to_kPa(valuex[4])
        psLd.value         = Volt_to_kPa(valuex[5])
        if switch.value == 0:
            FSRrh.value        = valuex[6]
            FSRrt.value        = valuex[7]
            FSRlh.value        = valuex[8]
            FSRlt.value        = valuex[9]

        
        # Record data to the file
        if record_start_fd.value == 1:                      
            if perf_counter() - record_interval > 0.009:        
                # print("here")
                dataA = [perf_counter() - c_time.value] +  [Lr.value] +  [Ll.value]  + [psRu.value] + [psRd.value] + [psLu.value] + [psLd.value] + [FSRrh.value] + [FSRrt.value] + [FSRlh.value] + [FSRlt.value]
                for ele in dataA:
                    fa.write(str(ele)+',')
                fa.write('\n')

                record_interval = perf_counter()
  
    fa.close()
    taskAI.stop()
    taskAI.close()


def Pressure_foot(Allexit,  FSRrh, FSRrt, FSRlh, FSRlt):
     
    Pressure_serial = serial.Serial('COM15', 115200, 
             parity=serial.PARITY_NONE,
             stopbits=serial.STOPBITS_ONE,
             bytesize=serial.EIGHTBITS, 
             timeout=.1)
    
    time.sleep(2)

    def read_pressure():
        line = Pressure_serial.readline().decode('utf-8').strip()  # decode bytes → string
        if line:  # make sure it's not empty
            try:
                parts = line.split()  # ['P1:991.865', 'P2:991.849', ...]
                values = {p.split(":")[0]: float(p.split(":")[1]) for p in parts}
                
                # Extract variables
                P1 = values.get("P1")
                P2 = values.get("P2")
                P3 = values.get("P3")
                P4 = values.get("P4")

                #print(f"P1: {P1}, P2: {P2}, P3: {P3}, P4: {P4}")
                return P1, P2, P3, P4
            except Exception as e:
                print(f"Parse error: {e}, line={line}")
    
    while not Allexit.value:
            Pressures = read_pressure()
            FSRrh.value, FSRrt.value, FSRlh.value, FSRlt.value = Pressures


    Pressure_serial.close()



## MAIN: Run the script #####################################################
if __name__ == "__main__":
    # mp. = multiple processinge
    
    
    switch = mp.Value('i', 0)    # 0 ----FSR, else Pressure Sensor
    heelR = mp.Value('i', 0) # right heel strike (HS) live state
    heelL = mp.Value('i', 0)
    toeR  = mp.Value('i', 0) # right toe strike (TS) live state
    toeL  = mp.Value('i', 0)
    HPeriod_available_R = mp.Value('i', 0) # check for three heel strike
    HPeriod_available_L = mp.Value('i', 0)
    HeelRp = mp.Value('d', 0) # average period between HS
    HeelLp = mp.Value('d', 0)
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
    Emg_sig = mp.Value('d', 0)  # trigger EMG

    Lr = mp.Value('d', 0)       # Loadcell value for right leg
    Ll = mp.Value('d', 0)
    psRu = mp.Value('d', 0)     # Pressure reading 1 
    psRd = mp.Value('d', 0)     # Pressure reading 2
    psLu = mp.Value('d', 0)
    psLd = mp.Value('d', 0)
    
    FSRrh = mp.Value('d', 0)    # FSR reading right heel (raw data)
    FSRrt = mp.Value('d', 0)
    FSRlh = mp.Value('d', 0)
    FSRlt = mp.Value('d', 0)
    c_time = mp.Value('d', 0)   # current time for DAQ

    timestrt = time.strftime("%Y%m%d-%H%M%S")
    LogFile = "All"+"_"+"Data"+"_"+timestrt+".csv"

## Execute Multiple Process ############################################################
    Daq = mp.Process(target=record_daq, args=(Allexit, record_start_fd, c_time, LogFile, Lr, Ll, psRu, psRd, psLu,psLd, FSRrh, FSRrt, FSRlh, FSRlt,switch ))
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
              supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn))
    # Timesforwalk = calculate gait period, actuation period 
    Walk_start = mp.Process(target=Walk, args=(heelR,heelL,toeR,toeL,\
        Allexit,\
        Rallow, Lallow, supportTimeR, unsupportTimeR, supportTimeL, unsupportTimeL,\
              supportTimeRAn, unsupportTimeRAn, supportTimeLAn,unsupportTimeLAn,\
                R, L, RR, LL, Emg_sig, r_pressed))
    # Walk = execute the actuation for walkings
   
    FSR.start() 
    print("FSR PID:", FSR.pid, "alive:", FSR.is_alive(), flush=True )
    # PID = process ID, alive = tell whether the function was executed.

    T_to_walk.start()
    print("T_to_walk PID:", T_to_walk.pid, "alive:", T_to_walk.is_alive(), flush=True )
    
    Walk_start.start()
    print("Walk start PID:", Walk_start.pid, "alive:", Walk_start.is_alive(), flush=True )

    Daq.start()   
    print("DAQ read start:", Daq.pid, "alive:", Daq.is_alive(), flush=True )

    if switch.value != 0:
         Foot_pressure.start()
         
         print("Foot Pressure Reading active:", Foot_pressure.pid, "alive:", Foot_pressure.is_alive(), flush=True )

    # Library for IMUs
    openzen.set_log_level(openzen.ZenLogLevel.Warning)

    #global Analogs
    #Analogs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]

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
    # s1 = 1
    # s2 = 1
    # s3 = 1
    # s4 = 1
    # s5 = 1
    t1now, t2now, t3now, t4now, t5now = 0,0,0,0,0
    # t1now = 0
    # t2now = 0
    # t3now = 0
    # t4now = 0
    # t5now = 0

    global rt, rs, lt, ls, hip
    rt, rs, lt, ls, hip = 90,90,90,90,90
    # rt = 90
    # rs = 90
    # lt = 90
    # ls = 90
    # hip = 90

    #MAX_SAMPLES = 500  # Number of samples to plot
    # data array for plotting
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
 
    # try = connect to all IMUs
    try:      
        (client, sensor1, imu1, sensor2, imu2, sensor3, imu3, sensor4, imu4, sensor5, imu5) = connectSensors()
        quit = False

        # If the connecton completes, ...
        data_thread = threading.Thread(target=data_acquisition, args=(client, imu1, imu2, imu3, imu4, imu5))
        data_thread.start()
        file_record()       # only create new files if connections are successful
        #thred_period.start()    # calculates FSR periods
        #thread_Process.start()  # calculate time of acutation
        #thread_walk.start()     # apply pressure based on time
        c_time.value = 0 # current time
        
        #apply_pressure(0, 0.1, 0.1, 0.1, 0.1)    
        mlocal = 0
        r_pressed.value = 0
        print("\nPress R to start recording or Q to Stop and E to exit")
        print("\n Right Heel - Toe - Left Heel - Toe - Right Thigh - Shank - Left Thigh - Shank - Hip")
        
        # Process the whole program
        while True:
            # for recording ########################################################################################
            if keyboard.is_pressed('r'):                
                r_pressed.value = 1
                if firsttime == 1:
                    c_time.value = perf_counter() # record the current time from perf_counter()
                    record_interval = copy.copy(c_time.value)
                    firsttime = 0
                    print("\n\nRecording started -- Press Q to Stop")
                    print("Right Heel - Toe - Left Heel - Toe - Right Thigh - Shank - Left Thigh - Shank - Hip")
                record_start  = 1
                record_start_fd.value = 1
            # print("here")
            

            ## Calculate FSR Threshold #################################################################################
            ### USE if switch.value is 0
            
            if switch.value == 0:  ##############   FSR
                if FSRrh.value> 1.5: #2.4:     #2.5
                    heelR.value = 0
                else: #1.9:   # 2
                    heelR.value = 1
                
                if FSRlh.value > 1: #2.4: #2.2
                    heelL.value = 0
                else: #1.9:  
                    heelL.value = 1

                if FSRlt.value> 2:    #3
                    toeL.value = 0
                else: #1.5
                    toeL.value = 1

                # to change to this one
                toeR.value = 0 if FSRrt.value > 1.5 else 1
            
            ########### Threshold for Foot Pressure ######
            #####    use this if switch.value is 1
            
            else:
                if FSRrh.value> 800: #2.4:     #2.5
                    heelR.value = 0
                else: #1.9:   # 2
                    heelR.value = 1
                
                if FSRlh.value > 900: #2.4: #2.2
                    heelL.value = 0
                else: #1.9:  
                    heelL.value = 1

                if FSRlt.value> 800:    #3
                    toeL.value = 0
                else: #1.5
                    toeL.value = 1

                toeR.value = 0 if FSRrt.value > 700 else 1
            
        # r = 0
                #valuex = copy.copy(Analogs)
                #print(buffer_in)   
                #break            
            #print("FSR PID:", FSR.pid, "alive:", FSR.is_alive(), flush=True )
    
            #print('     {:d},        {:d},       {:d},       {:d},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}'.format(int(heelR.value),  int(toeR.value), int(heelL.value),  int(toeL.value),float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end ='\r' ) 
            print('     {:f},        {:f},       {:f},       {:f},      {:.3f},     {:.3f},    {:.3f},    {:.3f}, {:.3f}'.format((FSRrh.value),  (FSRrt.value), (FSRlh.value),  (FSRlt.value),float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end ='\r' ) 
            
            # print('{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}'.format(float(IMU_1),  float(IMU_2), float(IMU_3),  float(IMU_4), float(IMU_5)), end = '\r' )    
            ### if you comment the above printing, you might need have a slight delay
           # time.sleep(0.0001)
           # print(perf_counter() - c_time)
            
            #if record_start == 1:                           
                    #if perf_counter() - record_interval > 0.009:        
                          #  dataA = [perf_counter() - c_time] +  [Lr] +  [Ll]  + [psRu] + [psRd] + [psLu] + [psLd] + [FSRrh] + [FSRrt] + [FSRlh] + [FSRlt]
                          #  for ele in dataA:
                          #          fa.write(str(ele)+',')
                          #  fa.write('\n')
                          #  record_interval = perf_counter()

            ## For quit the recording ###################################################################
            if keyboard.is_pressed('q') and r_pressed.value==1:
                        #taskAI.stop()
                        record_start  = 0
                        record_start_fd.value = 0
                        firsttime = 1
                
                        s1, s2, s3, s4, s5 = 1,1,1,1,1

                        #time.sleep(1)
                    # Analogs = taskAI.read()
                    # if r_pressed == 1:
                        print("\n Recording stopped")
                        r_pressed.value = 0
                                
                        # filing
                        mlocal = 0
                        print("\nPress R to record another trial or E to Exit")
                        #while True:
                        # Analogs = taskAI.read()


            ## Exit the whole program ###################################################################            
            if keyboard.is_pressed('e') and r_pressed.value == 0:
                                quit = True
                                Allexit.value = 1 # to quit all mp.process
                                mlocal = 1
                                break
                        
    ## If IMU connection fails, it will come here               
    except KeyboardInterrupt:
        pass
    ## If IMU connection succeeds and the program runs through, 
    finally:
        quit = True
        Allexit.value = 1 # to quit all mp.process
        data_thread.join()
        print ("\nStreaming of sensor data complete")
 

        # task_in.close()
    # fa.close()
    # sensor1.release()
    # sensor2.release()
    #  sensor3.release()
    #  sensor4.release()
    # sensor5.release()
    
        client.close()
        #  print("OpenZen client closed. Bye")

       # fa.close()
        f1.close()
        f2.close()
        f3.close()
        f4.close()
        f5.close()

        # To exit mp.process
        FSR.join()
        T_to_walk.join()
        Walk_start.join()
        Daq.join()
        
        if switch.value != 0:
             Foot_pressure.join()
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
        """ 
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
        LogFile = os.path.join(base_dir, f"All_Data_{timestr}.csv")
        if(os.path.exists(LogFile) and os.path.isfile(LogFile)):
                            os.remove(LogFile)
        """

   # main()
    

