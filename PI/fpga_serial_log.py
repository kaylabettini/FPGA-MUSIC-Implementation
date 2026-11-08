#!/usr/bin/env python3
"""
fpga_serial_log.py - log the HFMA angle estimates the FPGA sends over USB-UART

The FPGA (angle_uart_reporter.vhd) sends one ASCII line per report, e.g. "-60".
This logs every line with a timestamp to a CSV:

    t_s,angle_deg
    0.0213,30
    0.0412,29
    ...

Use it two ways:

1) Inside a streaming script (shared clock with the audio stream):
       from fpga_serial_log import FpgaLogger
       log = FpgaLogger("/dev/ttyUSB1", "30deg_speech_t1_fpga.csv")
       log.start()            # or log.start(t0) to share a perf_counter() start time
       ... stream for 30 s ...
       log.stop()             # closes the file and prints mean / median / std

2) On its own:
       python fpga_serial_log.py --port /dev/ttyUSB1 --duration 30 --log test.csv

Setup on the Pi (once):
    sudo apt install python3-serial        # or: pip install pyserial --break-system-packages
    sudo usermod -aG dialout $USER         # then log out and back in
    ls /dev/ttyUSB*                        # the Nexys 4 UART is usually /dev/ttyUSB1
"""

import argparse
import statistics
import threading
import time


class FpgaLogger:
    def __init__(self, port, path, baud=115200, skip_s=2.0):
        self.port, self.path, self.baud, self.skip_s = port, path, baud, skip_s
        self.latest = None          # most recent angle (for status prints)
        self.count = 0
        self.bad = 0
        self._readings = []         # (t, angle)
        self._stop = threading.Event()
        self._thread = None

    def start(self, t0=None):
        import serial               # imported here so the streamer runs without pyserial
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self._ser.reset_input_buffer()      # drop anything queued before this trial
        self._f = open(self.path, "w")
        self._f.write("t_s,angle_deg\n")
        self.t0 = time.perf_counter() if t0 is None else t0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"Logging FPGA estimates from {self.port} to {self.path}")

    def _run(self):
        first = True
        while not self._stop.is_set():
            raw = self._ser.readline()
            if not raw:
                continue
            t = time.perf_counter() - self.t0
            if first:               # may be a partial line if we started mid-byte
                first = False
                if not raw.endswith(b"\n"):
                    continue
            try:
                a = int(raw.decode("ascii").strip())
            except (UnicodeDecodeError, ValueError):
                self.bad += 1
                continue
            self.latest = a
            self.count += 1
            self._readings.append((t, a))
            self._f.write(f"{t:.4f},{a}\n")

    def stop(self):
        if not self._thread:
            return
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._f.close()
        self._ser.close()
        self._thread = None
        self.summary()

    def summary(self):
        kept = [a for t, a in self._readings if t >= self.skip_s]
        msg = f"FPGA log: {self.count} readings ({self.bad} bad lines)"
        if len(kept) >= 2:
            msg += (f" | after {self.skip_s:.0f} s: n={len(kept)} mean {statistics.mean(kept):+.1f}"
                    f" median {statistics.median(kept):+.1f} std {statistics.stdev(kept):.1f}")
        elif self.count == 0:
            msg += "  <- nothing received: check the cable, port name and that the UART build is loaded"
        print(msg)


def main():
    p = argparse.ArgumentParser(description="Log HFMA angle estimates from the FPGA UART")
    p.add_argument("--port", default="/dev/ttyUSB1")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--log", default="fpga_log.csv")
    p.add_argument("--duration", type=float, default=30.0)
    a = p.parse_args()
    log = FpgaLogger(a.port, a.log, a.baud)
    log.start()
    try:
        t_end = time.perf_counter() + a.duration
        while time.perf_counter() < t_end:
            time.sleep(1.0)
            print(f"  {log.count:5d} readings | latest {log.latest}")
    except KeyboardInterrupt:
        pass
    log.stop()


if __name__ == "__main__":
    main()