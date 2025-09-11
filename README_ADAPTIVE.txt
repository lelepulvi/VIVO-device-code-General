
a few changes in the main

-having a switch for IMU on/off
-Actuation is considering the previous heel strikes, and new functions added


In the Adaptive FSR, I changed the following (< to > and > to <)

   if self.state == 0:  # no-contact
      if v **>**= self.lower and dt_since_change >= self.min_dwell:
   else:  # contact
      if v **<**= self.upper and dt_since_change >= self.min_dwell:

init_contact=None, init_nocontact=None, --------> init_contact=0.2, init_nocontact=1.0




# README — Using Adaptive Thresholding in FREEHAB_FSR_Pressure.py

## Purpose
This guide explains how to replace fixed FSR/pressure thresholds with the **AdaptiveFSR** class, so thresholds self-adjust step by step during walking.

---

## Steps

### 1. Import the class
At the top of `FREEHAB_FSR_Pressure.py`, add:
```python
from adaptive_fsr import AdaptiveFSR
import time
```

---

### 2. Create detectors
After your `mp.Value(...)` definitions (once per channel):
```python
det_rh = AdaptiveFSR()  # Right Heel
det_rt = AdaptiveFSR()  # Right Toe
det_lh = AdaptiveFSR()  # Left Heel
det_lt = AdaptiveFSR()  # Left Toe
```

---

### 3. Replace thresholds

#### FSR mode (line ~893, when `switch.value == 0`)
Replace the fixed threshold code with:
```python
now = time.perf_counter()
heelR.value, *_ = det_rh.update(FSRrh.value, now)
toeR.value,  *_ = det_rt.update(FSRrt.value, now)
heelL.value, *_ = det_lh.update(FSRlh.value, now)
toeL.value,  *_ = det_lt.update(FSRlt.value, now)
```

#### Foot Pressure mode (line ~915, when `switch.value == 1`)
Do the same replacement:
```python
now = time.perf_counter()
heelR.value, *_ = det_rh.update(FSRrh.value, now)
toeR.value,  *_ = det_rt.update(FSRrt.value, now)
heelL.value, *_ = det_lh.update(FSRlh.value, now)
toeL.value,  *_ = det_lt.update(FSRlt.value, now)
```

---

### 4. Tuning (optional)
You can tweak detector parameters when creating them:
```python
det_rh = AdaptiveFSR(
    ema_alpha=0.25,       # faster adaptation
    hysteresis_frac=0.10, # wider/narrower hysteresis band
    min_contact_ms=60,
    min_release_ms=60,
    min_dwell_ms=120
)
```

---

## Notes
- Use **one detector per channel** (RH, RT, LH, LT).  
- The detector needs ~250 ms warmup before thresholds stabilise.  
- Output (`heelR.value`, etc.) remains **1 = contact, 0 = no-contact**, so the rest of the pipeline is unchanged.  
- Safe to run at high sampling rates.  

---

✅ With these changes, FREEHAB automatically adapts its thresholds and no longer requires manual tuning for each subject or session.  
