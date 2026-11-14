# todo
# bias: changes in sensitivity to change in light (diff on (19-50) vs diff off (26-60))
# find command to set event rate control
# digital event mask: cuts out hot pixels to lower bandwidth
# - see which pixel has abnormally high number within a 20ms period
# genx320 -> can use label data to recognize people?

# GENX320 event mode: measure event counts in CONSECUTIVE 20 ms windows.
# Board: OpenMV H7 Plus / RT1062 / N6 with GENX320, firmware v5.0+.
#
# Measurement method
#   - Windows sit on a fixed time grid: [T0 + k*20 ms, T0 + (k+1)*20 ms),
#     where T0 is the timestamp of the first event received. No gaps, no overlap.
#   - Events past the current window boundary are carried into the next window
#     (nothing is discarded at boundaries).
#   - Intervals with no events are recorded as zero-event windows.
#   - Every event is COUNTED, even if the storage buffer is full. Events that do
#     not fit are reported as "dropped" and the window is flagged as overflow,
#     so the counts and rates stay correct.
#   - N_WINDOWS consecutive windows therefore cover exactly N_WINDOWS x 20 ms
#     of sensor time (250 windows = 5.000 s).
#
# Outputs (on the camera drive / SD card)
#   window_log.csv        one row per window: index, start time, event count,
#                         stored, dropped, overflow flag
#   peak_window_20ms.bin  the stored events of the window with the highest count
#                         (12 bytes per event: type, s, ms, us, x, y)

import csi
import gc
import image
import time
from ulab import numpy as np

# ---------------- settings ----------------
TEST_NAME = "test1"     # label for this run (printed in the summary)
WINDOW_US = 20_000      # window length: 20 ms
N_WINDOWS = 250         # consecutive windows to measure (250 = 5 s)
READ_RES = 4096        # events per driver read (power of two, 1024-65536)
MAX_EVENTS = 20000       # storage limit per window (12 bytes each -> 240 KB)
LIVE_VIEW = True        # draw windows in the Frame Buffer while measuring
LIVE_EVERY = 5           # draw every 5th window (10 pictures/s) to keep the loop fast
SAVE_FILES = True        # write window_log.csv and peak_window_20ms.bin
# ------------------------------------------

gc.collect()  # start with as much free memory as possible
events = np.zeros((READ_RES, 6), dtype=np.uint16)     # one driver read
window = np.zeros((MAX_EVENTS, 6), dtype=np.uint16)   # current window storage
peak = np.zeros((MAX_EVENTS, 6), dtype=np.uint16)   # copy of the peak window
img = image.Image(320, 320, image.GRAYSCALE) if LIVE_VIEW else None

csi0 = csi.CSI(cid=csi.GENX320)
csi0.reset()
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, READ_RES)
csi0.framebuffers(10)  # queue reads instead of dropping them
# csi0.ioctl(csi.IOCTL_GENX320_SET_BIAS, csi.GENX320_BIAS_DIFF_OFF, 100)
csi0.ioctl(csi.IOCTL_GENX320_SET_AFK, 0)  # anti-flicker OFF (all events kept)
# to turn it back on:
# csi0.ioctl(csi.IOCTL_GENX320_SET_AFK, 1, 90, 150)


def t_us(row):
    # columns: [0] type, [1] s, [2] ms, [3] us, [4] x, [5] y
    return int(row[1]) * 1_000_000 + int(row[2]) * 1000 + int(row[3])


def first_index_at_or_after(buf, lo, hi, t_end):
    # events are time-ordered -> binary search in buf[lo:hi] for first time >= t_end
    while lo < hi:
        mid = (lo + hi) // 2
        if t_us(buf[mid]) < t_end:
            lo = mid + 1
        else:
            hi = mid
    return lo


# ---------------- measurement state ----------------
t0 = None          # start of the time grid (first event)
win_idx = 0        # index of the current window
win_end = 0        # end time of the current window (us)
stored = 0         # events stored in the current window
total = 0          # events counted in the current window (stored + dropped)

log = []           # (index, start_us, total, stored, dropped, overflow)
peak_total = -1
peak_stored = 0
peak_idx = -1


def close_window():
    global win_idx, win_end, stored, total, peak_total, peak_stored, peak_idx
    global window, peak
    dropped = total - stored
    overflow = 1 if dropped > 0 else 0
    log.append((win_idx, win_idx * WINDOW_US, total, stored, dropped, overflow))

    if LIVE_VIEW and win_idx % LIVE_EVERY == 0:
        img.draw_event_histogram(window[:stored], clear=True, brightness=128, contrast=64)
        img.flush()

    if total > peak_total:
        peak_total, peak_stored, peak_idx = total, stored, win_idx
        # swap buffers instead of copying: no extra memory needed.
        # The current window becomes the peak; the old peak buffer is reused.
        window, peak = peak, window

    # if (win_idx + 1) % 50 == 0:  # progress line every second
    #     print("window %d/%d: %d events" % (win_idx + 1, N_WINDOWS, total))

    # if overflow:
    #     print("window %d: overflow, %d events counted, %d dropped" % (win_idx, total, dropped))

    win_idx += 1
    win_end += WINDOW_US
    stored = 0
    total = 0


print("Measuring %d consecutive windows of %d ms (%s)..." % (N_WINDOWS, WINDOW_US // 1000, TEST_NAME))
csi0.ioctl(csi.IOCTL_GENX320_SET_MODE, csi.GENX320_MODE_EVENT, READ_RES)
# time.sleep_ms(500)  # let the sensor settle

while win_idx < N_WINDOWS:
    num_events = csi0.ioctl(csi.IOCTL_GENX320_READ_EVENTS, events)
    if num_events == 0:
        continue
    if t0 is None:
        t0 = t_us(events[0])
        win_end = t0 + WINDOW_US

    curr_win_idx = 0
    if curr_win_idx < num_events:
        print(num_events)
    while curr_win_idx < num_events and win_idx < N_WINDOWS:
        # events[curr_win_idx:least_larger_winend] belong to the current window
        least_geq_winend = first_index_at_or_after(events, curr_win_idx, num_events, win_end)
        cur_win_ct = least_geq_winend - curr_win_idx
        if cur_win_ct > 0:
            c = min(cur_win_ct, MAX_EVENTS - stored)
            if c > 0:
                window[stored:stored + c, :] = events[curr_win_idx:curr_win_idx + c, :]
                stored += c
            total += cur_win_ct          # count every event, stored or not
        curr_win_idx = least_geq_winend
        if least_geq_winend < num_events:
            # an event at/after win_end exists -> the current window is complete.
            # Repeats for every empty 20 ms interval up to that event.
            close_window()
    # if k == n, the window may still receive events from the next read

# ---------------- results ----------------
counts = [r[2] for r in log]
zero_windows = sum(1 for c in counts if c == 0)
overflow_windows = sum(r[5] for r in log)
dropped_total = sum(r[4] for r in log)
lo, hi = min(counts), max(counts)
avg = sum(counts) // len(counts)
rate = int(sum(counts) / (len(log) * WINDOW_US / 1e6))

print("")
print("=== %s: %d consecutive windows = %.3f s ===" % (TEST_NAME, len(log), len(log) * WINDOW_US / 1e6))
print("min : %6d events  %7d bytes  (raw %6d bytes)" % (lo, lo * 12, lo * 4))
print("avg : %6d events  %7d bytes  (raw %6d bytes)" % (avg, avg * 12, avg * 4))
print("max : %6d events  %7d bytes  (raw %6d bytes)  in window %d" % (hi, hi * 12, hi * 4, peak_idx))
print("zero-event windows : %d" % zero_windows)
print("overflow windows   : %d (MAX_EVENTS = %d)" % (overflow_windows, MAX_EVENTS))
print("events dropped     : %d" % dropped_total)
print("avg event rate     : %d ev/s" % rate)

if SAVE_FILES:
    with open("window_log.csv", "w") as f:
        f.write("window,start_us,events,stored,dropped,overflow\n")
        for r in log:
            f.write("%d,%d,%d,%d,%d,%d\n" % r)
    window = None   # free the working buffer before writing
    gc.collect()
    with open("peak_window_20ms.bin", "wb") as f:
        f.write(memoryview(peak.tobytes())[:peak_stored * 12])
    print("Saved window_log.csv and peak_window_20ms.bin (%d events)" % peak_stored)
