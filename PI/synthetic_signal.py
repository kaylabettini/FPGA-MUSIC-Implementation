#!/usr/bin/env python3
"""
make_synthetic.py - recreate the synthetic test files without the Pi

Writes synthetic/syn_<signal>_<angle>deg_sent.wav, sample-for-sample identical
to what USB-to-ethernet.py --mode 2 sent to the FPGA (and saved with --dump):
the same synthetic_blocks() code, the same seed, a 4 s buffer looped, int16, 8 kHz, 4 ch.

Usage:   python3 make_synthetic.py            (needs numpy)
         python3 make_synthetic.py 15         (length in seconds, default 30)
"""
import math, os, sys, wave
import numpy as np

RATE = 8000
FRAMES_PER_BLOCK = 64
MIC_SPACING = 0.042
SOUND_SPEED = 343.0

def synthetic_blocks(angle_deg, tones=None, spacing=MIC_SPACING, c=SOUND_SPEED,
                     seconds=4, level=0.3, seed=1):
    # copied unchanged from USB-to-ethernet.py (returns the whole buffer instead of yielding blocks)
    n = int(seconds * RATE) // FRAMES_PER_BLOCK * FRAMES_PER_BLOCK
    f = np.fft.rfftfreq(n, 1 / RATE)
    if tones:
        S = np.zeros(len(f), complex)
        for ft in tones:
            S[np.argmin(np.abs(f - ft))] = 1.0
    else:
        S = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
        S[(f < 300) | (f > 3400)] = 0
    tau = np.arange(4) * spacing * math.sin(math.radians(angle_deg)) / c
    sig = np.stack([np.fft.irfft(S * np.exp(-2j * np.pi * f * t), n) for t in tau], axis=1)
    sig *= level * 32767 / np.abs(sig).max()
    return np.round(sig).astype(np.int16)

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
os.makedirs("synthetic", exist_ok=True)
signals = {"noise": None, "600": [600], "1000": [1000], "1400": [1400], "2200": [2200], "3000": [3000]}
for name, tones in signals.items():
    for a in (-90, -60, -30, 0, 30, 60, 90):
        buf = synthetic_blocks(a, tones)
        N = int(seconds * RATE) // FRAMES_PER_BLOCK * FRAMES_PER_BLOCK
        x = np.tile(buf, (N // len(buf) + 1, 1))[:N]          # looped, as the Pi streams it
        ang = f"m{-a}" if a < 0 else str(a)
        path = os.path.join("synthetic", f"syn_{name}_{ang}deg_sent.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(4); w.setsampwidth(2); w.setframerate(RATE)
            w.writeframes(x.astype("<i2").tobytes())
        print("wrote", path)
print("Done: 42 files in ./synthetic")
