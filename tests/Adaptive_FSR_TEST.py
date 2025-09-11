import time
import math
from collections import deque
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# =========================
# TEST FUNCTION (middle)
# =========================
def test_adaptive_on_csv(
    csv_path,
    time_col=0,            # index or name; set None if no time column
    sensor_col=5,          # index (0-based) or name of FSR/pressure column
    smoothing_win=0,       # 0 = off
    fs=None,               # Hz if no/invalid time column
    out_prefix=None,
    show_plot=True
):
    csv_path = Path(csv_path)

    # Load
    df = pd.read_csv(csv_path, header=None, low_memory=False)

    # Time vector
    if time_col is not None:
        t_series = df.iloc[:, time_col] if isinstance(time_col, int) else df[time_col]
        t = pd.to_numeric(t_series, errors="coerce").to_numpy()
        if np.any(np.diff(t) < 0) or not np.isfinite(np.nanmedian(np.diff(t))):
            if fs is None:
                raise ValueError("Time column not monotonic; provide fs (Hz) or set time_col=None.")
            t = np.arange(len(df)) / float(fs)
        # Optionally convert ms→s if needed:
        # if np.nanmedian(np.diff(t)) > 5: t = t / 1000.0
    else:
        if fs is None:
            raise ValueError("No time_col; please set fs (Hz).")
        t = np.arange(len(df)) / float(fs)

    # Sensor column
    if isinstance(sensor_col, int):
        v = pd.to_numeric(df.iloc[:, sensor_col], errors="coerce").to_numpy()
        sensor_tag = f"col{sensor_col}"
    else:
        v = pd.to_numeric(df[sensor_col], errors="coerce").to_numpy()
        sensor_tag = sensor_col.replace(" ", "_")

    # Optional smoothing
    if smoothing_win and smoothing_win > 1:
        k = int(smoothing_win)
        v = np.convolve(v, np.ones(k)/k, mode="same")

    # Run detector
    det = AdaptiveFSR(hysteresis_frac=0.10, min_contact_ms=50, min_release_ms=50, min_dwell_ms=100)
    det.last_change_t = t[0] if len(t) else 0.0

    state = np.zeros_like(v, dtype=int)
    lower = np.full_like(v, np.nan, dtype=float)
    mid   = np.full_like(v, np.nan, dtype=float)
    upper = np.full_like(v, np.nan, dtype=float)

    for i, (vi, ti) in enumerate(zip(v, t)):
        s, lo, md, up = det.update(vi, ti)
        state[i] = s
        lower[i] = lo if lo is not None else np.nan
        mid[i]   = md if md is not None else np.nan
        upper[i] = up if up is not None else np.nan

    # Events
    trans = np.diff(state, prepend=state[0])
    contact_idx = np.where(trans == 1)[0]
    release_idx = np.where(trans == -1)[0]

    # Stats
    step_times = np.diff(t[contact_idx]) if len(contact_idx) > 1 else np.array([])
    cadence_spm = 60.0 / np.median(step_times) if step_times.size else np.nan

    print(f"Samples: {len(v)}")
    print(f"Detected contacts: {len(contact_idx)}, releases: {len(release_idx)}")
    if step_times.size:
        print(f"Median step time: {np.median(step_times):.3f} s  |  Cadence: {cadence_spm:.1f} steps/min")

    # Save
    out_prefix = out_prefix or (csv_path.with_suffix("").name + f"__{sensor_tag}")
    out_csv = csv_path.with_name(out_prefix + "_adaptive_output.csv")
    out_df = pd.DataFrame({"Time": t, "Signal": v, "State": state, "Lower": lower, "Mid": mid, "Upper": upper})
    out_df.to_csv(out_csv, index=False)
    print(f"Saved: {out_csv}")

    # Plot
    if show_plot:
        plt.figure(figsize=(12, 5))
        plt.plot(t, v, label="Signal")
        plt.plot(t, mid, label="Mid", linewidth=1)
        plt.plot(t, lower, label="Lower", linewidth=1)
        plt.plot(t, upper, label="Upper", linewidth=1)
        in_contact = state.astype(bool)
        start = None
        for i, c in enumerate(in_contact):
            if c and start is None:
                start = t[i]
            if (not c or i == len(in_contact) - 1) and start is not None:
                end = t[i] if not c else t[-1]
                plt.axvspan(start, end, alpha=0.08)
                start = None
        plt.title(f"Adaptive FSR on '{sensor_tag}'")
        plt.xlabel("Time (s)")
        plt.ylabel("Signal")
        plt.legend(loc="best")
        plt.tight_layout()
        plt.show()

    return out_csv, contact_idx, release_idx


# =========================
# MAIN (bottom)
# =========================
if __name__ == "__main__":
    # >>> EDIT THESE <<<
    CSV_PATH   = "C:/Users/ep15603/OneDrive - University of Bristol/Desktop/VIVO hub/4. ICEE.space project/2. EMG data/HEXsuit + IS1 test/SBJ5/SBJ_5_IS1On_HEXOn.csv"
    TIME_COL   = 0        # set to None if no time column
    SENSOR_COL = 5        # 0-based index of FSR/pressure signal
    FS_HZ      = None     # e.g., 1000 if TIME_COL is None or invalid
    SMOOTH_WIN = 5        # 0 = off
    SHOW_PLOT  = True

    print(">>> Running adaptive FSR tester...")
    out_csv, contacts, releases = test_adaptive_on_csv(
        csv_path=CSV_PATH,
        time_col=TIME_COL,
        sensor_col=SENSOR_COL,
        fs=FS_HZ,
        smoothing_win=SMOOTH_WIN,
        out_prefix=None,
        show_plot=SHOW_PLOT
    )
    print(">>> Done.")
    print(f">>> Output: {out_csv}")
    print(f">>> Contacts: {len(contacts)}  Releases: {len(releases)}")
