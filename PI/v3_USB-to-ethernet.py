import queue
import struct
import threading
import time
import wave
import argparse
import socket
import sounddevice as sd
import numpy as np
import math

##################################################
#             CONFIGURATION DEFAULTS             #
##################################################

DEVICE             = "hw:0,0"
CHANNELS           = 16
RATE               = 8000
FRAMES_PER_BLOCK   = 64
SAMPLE_DTYPE       = "int32"
SAMPLE_WIDTH_BYTES = 4
MAX_CHUNK = 1024

SAVE_WAVE          = "saved_audio.wav"
OPEN_WAVE          = "open_audio.wav"

UDP_IP             = "192.168.1.255"  #Defaults to the loopback which pretty mc
UDP_PORT           = 5000

#Synthetic mode (mode 2)
MIC_SPACING        = 0.042   #m, matches the model and the UMA-16 rows
SOUND_SPEED        = 343.0   #m/s

#FPGA estimate logging (--serial)
SERIAL_PORT        = "/dev/ttyUSB1"   #Nexys 4 USB-UART (ttyUSB0 is the JTAG side)
SERIAL_BAUD        = 115200
FPGA_LOG           = "fpga_log.csv"

##################################################
#                     QUEUES                     #
##################################################

ethernet_q = queue.Queue(maxsize=8)
wav_q      = queue.Queue(maxsize=32)

##################################################
#                     GLOBALS                    #
##################################################

running  = True
sequence = 0
quiet    = False

fpga_log    = None    #FpgaLogger when --serial is given
log_started = False
dump_wf     = None    #wave writer for --dump (the exact 4 ch int16 stream sent to the FPGA)
stats       = {"peak": 0, "clipped": 0}

##################################################
#                UDP FUNCTIONS                   #
##################################################

"""
Handles making the header for the file
Sync word once again is 0xF00DA55A
"""
def make_header(seq: int, frag: int, total_frags: int) -> bytes:
    """
    Bytes 0..3:  sync word      0xF00DA55A (BE)
    Bytes 4..7:  sequence number           (BE)
    Byte  8:     fragment id
    Byte  9:     fragment total
    Total: 10 bytes
    """
    return struct.pack(">IIBB", 0xF00DA55A, seq, frag, total_frags)

"""
Helper function for splitting the data over separate UDP
transmissions as all the data cannot be inputted into one
packet
"""
def send_block_udp(sock, seq, pcm_bytes):
    total     = len(pcm_bytes)
    num_frags = math.ceil(total / MAX_CHUNK)

    for frag in range(num_frags):
        chunk  = pcm_bytes[frag * MAX_CHUNK : (frag + 1) * MAX_CHUNK]
        header = make_header(seq, frag, num_frags)
        packet = header + chunk
        #packet = struct.pack(">IIBB", 0x0DF05AA5, 0x12345678, 0x01, 0x02) + bytes([0xAA] * 1024)
        sock.sendto(packet, (UDP_IP, UDP_PORT))

"""
Sends one (64, 4) int16 block, used by all three modes.
- starts the FPGA log on the very first packet, so the log's t = 0 is the
  first packet sent (lines up with --dump and the saved WAV)
- keeps the peak level for the status line
- writes the block to the --dump WAV
"""
def send_block(sock, seq, sel):
    global log_started
    if fpga_log and not log_started:
        fpga_log.start()
        log_started = True
    pcm_bytes = sel.tobytes(order="C")
    send_block_udp(sock, seq, pcm_bytes)
    stats["peak"] = max(stats["peak"], int(np.abs(sel.astype(np.int32)).max()))
    if dump_wf:
        dump_wf.writeframes(pcm_bytes)

def print_status(seq):
    if quiet:
        return
    msg = f"seq {seq:6d} | peak so far {stats['peak']:5d} | clipped {stats['clipped']}"
    if fpga_log:
        msg += f" | FPGA {fpga_log.latest} ({fpga_log.count} readings)"
    print(msg)

##################################################
#                    CALLBACKS                   #
##################################################

"""
This is the function that gets called when sounddevice
receives an audio signal incoming.
This callback handles queueing the incoming data for both
the ethernet queue and the wav saver queue
"""
def audio_callback(indata, frames, time_info, status):
    if status:
        print("Audio status:", status)
    if frames != FRAMES_PER_BLOCK:
        print(f"Unexpected frame count: {frames}")

    block = indata.copy()

    try:
        ethernet_q.put_nowait(block)
    except queue.Full:
        print("Ethernet queue full: dropping block")

    try:
        wav_q.put_nowait(block)
    except queue.Full:
        print("WAV queue full: dropping block")

##################################################
#                 THREAD WORKERS                 #
##################################################

"""
Handles saving the recorded audio to a .wav
file for storage purposes and playback later
"""
def wav_worker(filename):
    global running

    with wave.open(filename, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH_BYTES)
        wf.setframerate(RATE)

        while running or not wav_q.empty():
            try:
                block = wav_q.get(timeout=0.2)
            except queue.Empty:
                continue
            wf.writeframes(block.tobytes(order="C"))

"""
Ethernet worker thread
Handles starting the UDP connection through socket and then
handles sending UDP blocks over the thread
Stops everything after max_blocks packets (--duration), if given
"""
def ethernet_worker(cols: list, gain: float = 1, max_blocks=None):
    global running, sequence

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP worker ready | target={UDP_IP}:{UDP_PORT}")

    while running or not ethernet_q.empty():
        try:
            block = ethernet_q.get(timeout=0.2)
        except queue.Empty:
            continue

        sel = process_block(block, cols, gain);
        send_block(sock, sequence, sel)
        sequence = (sequence + 1) & 0xFFFFFFFF

        if max_blocks and sequence >= max_blocks:
            print(f"Duration reached ({sequence} packets).")
            running = False
            break

    sock.close()

##################################################
#                    FUNCTIONS                   #
##################################################

def process_block(block, cols: list, gain: float = 1):
    sel = block[:, cols]
    sel = sel * gain/65536
    stats["clipped"] += int(np.count_nonzero(np.abs(sel) > 32767))
    sel = sel.clip(-32768, 32767).astype(np.int16)
    return sel

def arg_parsing():
    parser = argparse.ArgumentParser(description="USB-to-ethernet bridge")
    parser.add_argument("--device",   type=str, default=DEVICE)
    parser.add_argument("--channels", type=int, default=CHANNELS)
    parser.add_argument("--mics", type=int, nargs=4, default=[4, 3, 14, 13],
                    help="MIC labels printed on the board")
    parser.add_argument("--ip",       type=str, default=UDP_IP)
    parser.add_argument("--port",     type=int, default=UDP_PORT)
    parser.add_argument("--no_print", action="store_true", default=False)
    parser.add_argument("--mode", type=int, default=1) #0 for playback, 1 for live, 2 for synthetic
    parser.add_argument("--gain", type=float, default=1) #gain that the user wants to apply to the signal, default is 1 (no gain)
    parser.add_argument("--save_wav", type=str, default=SAVE_WAVE) #filename for the wav file to save
    parser.add_argument("--open_wav", type=str, default=OPEN_WAVE) #filename for the wave to open for playback
    #Synthetic mode (mode 2)
    parser.add_argument("--angle", type=float, default=30.0) #source angle in degrees
    parser.add_argument("--tones", type=float, nargs="*", default=None) #e.g. --tones 1400, or --tones 600 1400 2200 3000; leave out for noise
    parser.add_argument("--spacing", type=float, default=MIC_SPACING) #mic spacing in m
    parser.add_argument("--duration", type=float, default=None) #stop after this many seconds (all modes)
    #Trial logging
    parser.add_argument("--dump", type=str, default=None) #save the exact 4 ch 8 kHz int16 stream sent to the FPGA (for Simulink)
    parser.add_argument("--serial", nargs="?", const=SERIAL_PORT, default=None) #log the FPGA estimates, e.g. --serial or --serial /dev/ttyUSB1
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--log", type=str, default=FPGA_LOG) #CSV for the FPGA estimates
    return parser.parse_args()

"""
Playback loop is the second mode for the USB-to-Ethernet file
This handles playing back the same file mic_array_capture.wav
WAV->UDP and plays it exactly as received (Same timing)
"""
def playback_loop(cols, gain, open_wave: str = OPEN_WAVE, duration=None):

    #Setup the UDP connection socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1) #Multicast 239.1.1.1

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    #Sequence Number
    seq = 0

    with wave.open(open_wave, "rb") as wf:

        #Validating the .WAV file
        sample_width = wf.getsampwidth()
        assert sample_width == 4, f"Expected 4 bytes per sample, got {sample_width}"
        assert wf.getnchannels() == CHANNELS, f"Expected {CHANNELS} channels, got {wf.getnchannels()}"
        assert wf.getframerate() == RATE,     f"Expected {RATE} Hz, got {wf.getframerate()}"

        expected_payload_bytes = FRAMES_PER_BLOCK * CHANNELS * sample_width
        block_duration_s       = FRAMES_PER_BLOCK / RATE #64 / 8000 ≈ 8ms per packet
        max_blocks = int(duration / block_duration_s) if duration else None

        print(f"Replaying WAV | {wf.getnchannels()}ch | {sample_width*8}-bit | {wf.getframerate()}Hz | {wf.getnframes()} frames")

        t_next = time.perf_counter()
        t_status = t_next + 1.0

        try:
            while True:
                wf.rewind()
                while True:
                    payload = wf.readframes(FRAMES_PER_BLOCK)

                    #EOF
                    if len(payload) == 0:
                        break

                    if len(payload) < expected_payload_bytes:
                        payload += bytes(expected_payload_bytes - len(payload))

                    sel = process_block(np.frombuffer(payload, dtype=SAMPLE_DTYPE).reshape(-1, CHANNELS), cols, gain)
                    send_block(sock, seq, sel)
                    seq = (seq + 1) & 0xFFFFFFFF
                    if max_blocks and seq >= max_blocks:
                        print(f"Duration {duration}s reached.")
                        raise StopIteration

                    t_next += block_duration_s
                    sleep_time = t_next - time.perf_counter()
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    else:
                        print(f"Warning: behind by {-sleep_time*1000:.2f}ms")
                    if time.perf_counter() >= t_status:
                        print_status(seq)
                        t_status += 1.0

                print("EOF: Replaying...")
        except (StopIteration, KeyboardInterrupt):
            pass

    sock.close()

"""
Synthetic mode: a plane wave arriving at a 4-mic line array from a
known angle, so the FPGA reading can be checked against the true answer
without any acoustics.
Mic m (0..3) is delayed by m * spacing * sin(angle) / c relative to Mic1.
tones=None -> band-limited noise 300-3400 Hz
tones=[f1, f2, ...] -> sum of sines at those frequencies
Yields (64, 4) int16 blocks (Mic1..Mic4) forever. The buffer is 4 s long
and every tone has a whole number of cycles in it, so it loops seamlessly.
The signal is already int16 at about -10 dBFS peak, so no gain is applied.
"""
def synthetic_blocks(angle_deg, tones=None, spacing=MIC_SPACING, c=SOUND_SPEED,
                     seconds=4, level=0.3, seed=1):
    n = int(seconds * RATE) // FRAMES_PER_BLOCK * FRAMES_PER_BLOCK
    f = np.fft.rfftfreq(n, 1 / RATE)

    if tones:
        S = np.zeros(len(f), complex)
        for ft in tones:
            if not 0 < ft < RATE / 2:
                raise SystemExit(f"Tone {ft} Hz must be between 0 and {RATE/2:.0f} Hz at {RATE} Hz sampling")
            S[np.argmin(np.abs(f - ft))] = 1.0
    else:
        S = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
        S[(f < 300) | (f > 3400)] = 0

    tau = np.arange(4) * spacing * math.sin(math.radians(angle_deg)) / c
    sig = np.stack([np.fft.irfft(S * np.exp(-2j * np.pi * f * t), n) for t in tau], axis=1)
    sig *= level * 32767 / np.abs(sig).max()
    sig = np.round(sig).astype(np.int16)

    what = f"tones {tones} Hz" if tones else "noise 300-3400 Hz"
    print(f"Synthetic plane wave | {angle_deg:+.1f} deg | {what} | d={spacing*1000:.1f} mm | "
          f"delays (samples) = {np.round(tau * RATE, 3).tolist()}")

    while True:
        for b in range(0, n, FRAMES_PER_BLOCK):
            yield sig[b:b + FRAMES_PER_BLOCK]

"""
Sends the synthetic blocks one packet every 8 ms on a fixed schedule
(there is no sound card to set the timing, and sending too fast would
overflow the FPGA FIFO)
"""
def synthetic_loop(angle, tones, spacing, duration=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    block_duration_s = FRAMES_PER_BLOCK / RATE
    max_blocks = int(duration / block_duration_s) if duration else None
    seq = 0
    print(f"Sending synthetic stream to {UDP_IP}:{UDP_PORT}. Press Ctrl+C to stop.")

    try:
        t_next = None
        for block in synthetic_blocks(angle, tones, spacing):
            if t_next is None:
                t_next = time.perf_counter()   #start timing after the signal is generated
                t_status = t_next + 1.0
            send_block(sock, seq, block)
            seq = (seq + 1) & 0xFFFFFFFF
            if max_blocks and seq >= max_blocks:
                print(f"Duration {duration}s reached.")
                break

            t_next += block_duration_s
            sleep_time = t_next - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.1:            #fell far behind: resync
                t_next = time.perf_counter()
            if time.perf_counter() >= t_status:
                print_status(seq)
                t_status += 1.0
    except KeyboardInterrupt:
        print("Stopped.")

    sock.close()

"""
Live mode: record the array, send to the FPGA and save the 16 ch WAV
"""
def live_loop(args, cols):
    global running

    print("Available devices:")
    devices = sd.query_devices()
    print(devices if devices else "N/A")

    max_blocks = int(args.duration * RATE / FRAMES_PER_BLOCK) if args.duration else None
    ethernet_thread = threading.Thread(target=ethernet_worker, args=(cols, args.gain, max_blocks), daemon=True)
    wav_thread = threading.Thread(target=wav_worker, args=(args.save_wav,), daemon=True)
    wav_thread.start()
    ethernet_thread.start()

    try:
        with sd.InputStream(device=args.device, channels=args.channels, samplerate=RATE, blocksize=FRAMES_PER_BLOCK, dtype=SAMPLE_DTYPE, callback=audio_callback,):
            print("Streaming. Press Ctrl+C to stop.")
            t_status = time.perf_counter() + 1.0
            try:
                while running:                 #the ethernet worker clears this at --duration
                    time.sleep(0.1)
                    if time.perf_counter() >= t_status:
                        print_status(sequence)
                        t_status += 1.0
            except KeyboardInterrupt:
                print("Stopping...")
    except Exception as e:
        print("Audio device unavailable:", e)

    running = False
    ethernet_thread.join(timeout=2)
    wav_thread.join(timeout=2)

"""
Opens the --dump WAV and the --serial logger (if asked for)
"""
def open_trial_outputs(args):
    global fpga_log, dump_wf
    if args.dump:
        dump_wf = wave.open(args.dump, "wb")
        dump_wf.setnchannels(4)
        dump_wf.setsampwidth(2)
        dump_wf.setframerate(RATE)
        print(f"Saving the stream sent to the FPGA to {args.dump}")
    if args.serial:
        try:
            from fpga_serial_log import FpgaLogger
        except ImportError:
            raise SystemExit("--serial needs fpga_serial_log.py in the same folder as this script")
        fpga_log = FpgaLogger(args.serial, args.log, args.baud)

def close_trial_outputs():
    if fpga_log and log_started:
        fpga_log.stop()                         #prints n, mean, median and std
    if dump_wf:
        dump_wf.close()
    print(f"Levels: peak {stats['peak']} | clipped samples {stats['clipped']}")

def main():
    global UDP_IP, UDP_PORT, quiet

    args = arg_parsing()
    cols = [m - 1 for m in args.mics]
    UDP_IP   = args.ip
    UDP_PORT = args.port
    quiet    = args.no_print

    open_trial_outputs(args)
    try:
        if args.mode == 2: #synthetic plane wave, no microphones needed
            synthetic_loop(args.angle, args.tones, args.spacing, args.duration)
        elif args.mode == 1: #record live audio and send it and save it to a wav
            live_loop(args, cols)
        else: #Playback version
            playback_loop(cols, args.gain, args.open_wav, args.duration)
    finally:
        close_trial_outputs()


if __name__ == "__main__":
    main()