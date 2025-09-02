# VIVO-device-code-General
Main code for device control (walking, sit-to-stand, roam devices)

The FREEHAB_FSR_Pressure.py is the most recent file

In Line 696,

    switch = mp.Value('i', 0)  0 ----> IT will use FSR

if switch = mp.Value('i', 1)   1 ----> IT will use Foot pressure
 
Thresholds need to be set as well

IF FSR is being used, change thresholds in LINE 893
IF Foot Pressure is being used, change thresholds in LINE 915

