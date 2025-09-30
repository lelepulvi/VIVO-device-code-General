


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
    sensor2MacID = "00:04:3E:4B:31:C3"   # right shank
    sensor3MacID = "00:04:3E:86:27:24"   # left thigh
    sensor4MacID = "00:04:3E:86:27:9A"   # left shank
    sensor5MacID = "00:04:3E:86:27:E0"   # hip

    print ("Connecting to Sensors")
    # Connect to sensor1
    
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
    error = imu2.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu2.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor2: {}".format(freq))

    error = imu5.set_int32_property(openzen.ZenImuProperty.SamplingRate, streamFreq)
    error, freq = imu5.get_int32_property(openzen.ZenImuProperty.SamplingRate)
    print("Sampling rate sensor5: {}".format(freq))

    # Sync sensors
    print ("Sensors sync")
    imu2.execute_property(openzen.ZenImuProperty.StartSensorSync)
   
    imu5.execute_property(openzen.ZenImuProperty.StartSensorSync)
    # wait a moment for the synchronization commands to arrive
    time.sleep(3)

    # clear internal openzen event queue to remove existing data
    while True:
        zenEvent = client.poll_next_event()
        if zenEvent == None:
            break
    imu2.execute_property(openzen.ZenImuProperty.StopSensorSync)
    
    # set both sensors back to normal mode, sensor will start data streaming after these commands
    imu5.execute_property(openzen.ZenImuProperty.StopSensorSync)
    print("Sync completed")

    return (client,  sensor2, imu2,sensor5, imu5)

## Function: acquire IMUs (from Supplier's library) #####################################################
def data_acquisition(client, imu2,IMU_22,IMU_2z, imu5,IMU_55,IMU_5z,IMU_5zV):
    global imu1_data, imu1_dt, imu2_data, imu2_dt,imu3_data, imu3_dt,imu4_data, imu4_dt,imu5_data, imu5_dt, quit, f1,f2, f3, f4, f5
    
    global rt, rs, lt, ls, hip
    global record_start
    global s1, s2, s3, s4, s5, t1now, t2now ,t3now, t4now, t5now
    print("Data data_acquisition thread started")
    global LogFile1, LogFile2
    global IMU_1, IMU_2, IMU_3, IMU_4, IMU_5
    local = 0
    detected_time = perf_counter()
    
    while not quit:
        zenEvent = client.wait_for_next_event()

        # check for IMU1 event
        if zenEvent.event_type == openzen.ZenEventType.ImuData:
         if   zenEvent.sensor == imu2.sensor and \
            zenEvent.component.handle == imu2.component.handle:

            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
            IMU_2 = data
            IMU_22.value = data
            IMU_2z.value = copy.copy(imu_data.r[2])            
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
         if   zenEvent.sensor == imu5.sensor and \
            zenEvent.component.handle == imu5.component.handle:

            imu_data = zenEvent.data.imu_data
            data = copy.copy(imu_data.r[0])
           # dataV = copy.copy(imu_data.v[2])
            IMU_5 = data
            hip = data
            IMU_55.value = data
            IMU_5z.value  = copy.copy(imu_data.r[2])  
            #IMU_5zV.value = dataV         
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

## Function: calculate FSR period ####################################################
from time import perf_counter

def detect_shank_move(shank_angle, ref_angle, ref_time,
                      threshold=20.0, window=1.0, debug=False):
    """
    Detect shank movement:
    - Returns True if Δangle >= threshold within window.
    - Always refreshes reference.
    """
    now = perf_counter()
    moved = False
    last_move_time = None

    if shank_angle is not None:
        if ref_angle is None:
            ref_angle, ref_time = shank_angle, now
            if debug:
                print(f"[INIT] Shank ref set at {shank_angle:.1f}°")
        else:
            d_angle = abs(shank_angle - ref_angle)
            dt = now - ref_time
            if d_angle >= threshold:
                if dt <= window:
                    moved = True
                    last_move_time = now
                    if debug:
                        print(f"[MOVE] Δ={d_angle:.1f}° in {dt:.2f}s → WALK")
                ref_angle, ref_time = shank_angle, now
            elif dt > window:
                ref_angle, ref_time = shank_angle, now

    return moved, ref_angle, ref_time, last_move_time


def state_detection(IMU_angle, last_angle, unwrapped_angle,
                   heading_anchor, last_state, time_now, pressure,
                   swaylimit=12, stable_time=1.0,
                   stable_start=None, last_stable_angle=None,
                   shank_angle=None, shank_ref_angle=None, shank_ref_time=None,
                   shank_move_threshold=10.0, shank_window=1.0, gait_period=2.0,
                   last_move_time=None, debug=False):
    """
    State machine:
      - Walking (1): within sway, and movement detected within gait_period → pressure=0.4
      - Turning (0): outside sway → pressure=0
      - Standing (2): within sway, no movement for gait_period → pressure=0
    """

    # --- Unwrap IMU yaw ---
    dtheta = IMU_angle - last_angle
    if dtheta > 180: dtheta -= 360
    elif dtheta < -180: dtheta += 360
    new_unwrapped = unwrapped_angle + dtheta

    state = last_state
    new_pressure = pressure
    new_anchor = heading_anchor
    new_time_now = perf_counter()
    new_stable_start = stable_start
    new_last_stable_angle = last_stable_angle

    # --- Shank detection ---
    moved, new_shank_ref_angle, new_shank_ref_time, this_move_time = detect_shank_move(
        shank_angle, shank_ref_angle, shank_ref_time,
        threshold=shank_move_threshold, window=shank_window, debug=debug
    )
    if this_move_time is not None:
        last_move_time = this_move_time  # update persistent move time

    # --- Logic ---
    if last_state in (1, 2):  # walking or standing
        if abs(new_unwrapped - heading_anchor) > swaylimit:
            state, new_pressure = 0, 0
            new_stable_start = None
            new_last_stable_angle = new_unwrapped
            if debug: print(f"[TURN] Heading drift {abs(new_unwrapped - heading_anchor):.1f}°")
        else:
            if last_move_time and (new_time_now - last_move_time) <= gait_period:
                state, new_pressure = 1, 0.4
                if debug: print(f"[WALK] Last move {new_time_now - last_move_time:.2f}s ago")
            else:
                state, new_pressure = 2, 0
                if debug: print(f"[STAND] No move for > {gait_period:.1f}s")

    elif last_state == 0:  # turning
        if last_stable_angle is None:
            new_last_stable_angle = new_unwrapped
            new_stable_start = perf_counter()
        else:
            if abs(new_unwrapped - last_stable_angle) < 5.0:
                if new_stable_start is None:
                    new_stable_start = perf_counter()
                elif perf_counter() - new_stable_start >= stable_time:
                    if last_move_time and (new_time_now - last_move_time) <= gait_period:
                        state, new_pressure = 1, 0.4
                    else:
                        state, new_pressure = 2, 0
                    new_anchor = new_unwrapped
                    new_stable_start = None
                    new_last_stable_angle = None
                    if debug: print(f"[STABLE] Heading stable → state={state}")
            else:
                new_last_stable_angle = new_unwrapped
                new_stable_start = None

    return (new_unwrapped, IMU_angle, state, new_pressure,
            new_anchor, state, new_time_now, new_stable_start,
            new_last_stable_angle, new_shank_ref_angle, new_shank_ref_time,
            last_move_time)

if __name__ == "__main__":
    # mp. = multiple processinge
    ENABLE_LIVEPLOT = 0  # 0 will not plot live
    imu_on = 1              # 1 means IMU will be attempted to connect, 0 otherwise
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
    IMU_5zV= mp.Value('d', 0)


    FSRrh = mp.Value('d', 0)    # FSR reading right heel (raw data)
    FSRrt = mp.Value('d', 0)
    FSRlh = mp.Value('d', 0)
    FSRlt = mp.Value('d', 0)
    c_time = mp.Value('d', 0)   # current time for DAQ

    timestrt = time.strftime("%Y%m%d-%H%M%S")
    LogFile = "Turning"+"_"+"Data"+"_"+timestrt+".csv"


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
    
    firsttime = 1
    record_interval = 0   
    # try = connect to all IMUs
    openzen.set_log_level(openzen.ZenLogLevel.Warning)


    try:      
        if (imu_on):
          (client, sensor2, imu2, sensor5, imu5) = connectSensors()
          quit = False

        # If the connecton completes, ...
          data_thread = threading.Thread(target=data_acquisition, args=(client, imu2, IMU_22, IMU_2z, imu5, IMU_55,IMU_5z,IMU_5zV))
          data_thread.start()
              # only create new files if connections are successful
        c_time.value = 0 # current time
        
        #apply_pressure(0, 0.1, 0.1, 0.1, 0.1)    
        mlocal = 0
        r_pressed.value = 1
        last_angle = 0
        unwrapped_angle = 0
        heading_anchor = 0
        state = 2
        pressure = 0
        stable_start = None
        last_stable_angle = None
        shank_ref_angle = None
        shank_ref_time = None
        last_move_time = None


        while True:
            
            
            IMU_angle = copy.copy(IMU_5z.value)
            shank_angle = copy.copy(IMU_22.value)            
            result = state_detection(IMU_angle, last_angle, unwrapped_angle,
                                    heading_anchor, state, None, pressure,
                                    stable_start=stable_start,
                                    last_stable_angle=last_stable_angle,swaylimit=12, stable_time=2.5,
                                    shank_angle=shank_angle,
                                    shank_ref_angle=shank_ref_angle,
                                    shank_ref_time=shank_ref_time,
                                    shank_move_threshold=15.0,
                                    shank_window=1.0,
                                    gait_period=2.0,
                                    last_move_time=last_move_time,
                                    debug=False)

            (unwrapped_angle, last_angle, state, pressure,
            heading_anchor, _, time_now, stable_start,
            last_stable_angle, shank_ref_angle, shank_ref_time,
            last_move_time) = result
#print(f"Yaw={unwrapped_angle:.1f}, State={state}, Pressure={pressure}, Shank={shank_angle}")


            print(f"Yaw={unwrapped_angle:3.1f},  Heading={heading_anchor:3.1f}, Shank={shank_angle:3.1f}, State={state:1d}, Pressure={pressure:0.1f}", end="\r")
            #print(f"Yaw={unwrapped_angle:.1f}, Heading={heading_anchor:.1f}, "
             #   f"State={state}, Pressure={pressure}, Shank={shank_angle}")
   # print(f"Step {t} | State={state} | Pressure={pressure}")
           # print(f"Yaw={unwrapped_angle:.1f}, heading={heading_anchor:.2f}, State={state}, Pressure={pressure}, right shank = {IMU_2}")
           # yaw = copy.copy(IMU_5z.value)
           # print(f"State={state}, Pressure={pressure}, RefHeading={ref_heading:.1f}, Yaw={last_angle:.1f}")
         #   print(f"Initial heading anchor={heading_anchor:.2f}, current={unwrapped:.2f} state={ state}, pressure={Pressure:.2f} ")
       
            if keyboard.is_pressed('q') and r_pressed.value==1:
                        #taskAI.stop()
                        record_start  = 0
                        record_start_fd.value = 0
                        firsttime = 1
                
                        s1, s2, s3, s4, s5 = 1,1,1,1,1

               
                        print("\n Recording stopped")
                        r_pressed.value = 0
                                
                        # filing
                        mlocal = 0
                        print("\nPress R to record another trial or E to Exit")
                        #while True:
                        # Analogs = taskAI.read()


            ## Exit the whole program ###################################################################            
            if keyboard.is_pressed('e') and r_pressed.value == 1:
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
        if (imu_on):
            data_thread.join()
            print ("\nStreaming of sensor data complete")
            client.close()    
       