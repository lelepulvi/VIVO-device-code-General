# VIVO-device-code-General
Main code for device control (walking, sit-to-stand, roam devices)


################ FILE details ################   
The FREEHAB_FSR_Pressure.py is the most recent file. FREEHAB_FSR only uses FSR, whereas the newer file can use both.

In Line 696,

    switch = mp.Value('i', 0)  0 ----> IT will use FSR

if switch = mp.Value('i', 1)   1 ----> IT will use Foot pressure
 
Thresholds need to be set as well
You need to install nidaqmax and other python libraries. The COM port for Arduino and the pressure sensor Serial port may need to be changed



IF FSR is being used, change thresholds in LINE 893
IF Foot Pressure is being used, change thresholds in LINE 915

- openzen.pyd and SiUSBXP.dll are required to be placed in the same directory 
- Arduino is the C code for Arduino DUE

