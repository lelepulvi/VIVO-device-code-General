import time
import math
from collections import deque
# Libraries needed fot tester
#import numpy as np
#import pandas as pd
#import matplotlib.pyplot as plt
#from pathlib import Path


# =========================
# AdaptiveFSR CLASS (top)
# =========================
class AdaptiveFSR:
    def __init__(self, init_contact=None, init_nocontact=None,
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
            if v <= self.lower and dt_since_change >= self.min_dwell:
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
            if v >= self.upper and dt_since_change >= self.min_dwell:
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
