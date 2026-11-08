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

##################################################
#                UDP FUNCTIONS                   #
##################################################

"""
Handles making the header for the file
Sync word once again is 0xF00DA55A
"""
def make_header(seq: int, frag: int, total_frags: int) -> bytes:
    """
    Bytes 0..3:  sync word      0x0DF05AA5 (BE)
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
"""
def ethernet_worker(cols: list, gain: float = 1):
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
                
        pcm_bytes = sel.tobytes(order="C")
        send_block_udp(sock, sequence, pcm_bytes)
        sequence = (sequence + 1) & 0xFFFFFFFF

    sock.close()

##################################################
#                    FUNCTIONS                   #
##################################################

def process_block(block, cols: list, gain: float = 1):
    sel = block[:, cols]
    sel = sel * gain/65536
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
    parser.add_argument("--mode", type=int, default=1) #0 for playback, 1 for live
    parser.add_argument("--gain", type=float, default=1) #gain that the user wants to apply to the signal, default is 1 (no gain)
    parser.add_argument("--save_wav", type=str, default=SAVE_WAVE) #filename for the wav file to save
    parser.add_argument("--open_wav", type=str, default=OPEN_WAVE) #filename for the wave to open for playback
    return parser.parse_args()

"""
Playback loop is the second mode for the USB-to-Ethernet file
This handles playing back the same file mic_array_capture.wav
WAV->UDP and plays it exactly as received (Same timing)
"""
def playback_loop(cols, gain, open_wave: str = OPEN_WAVE):

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

        print(f"Replaying WAV | {wf.getnchannels()}ch | {sample_width*8}-bit | {wf.getframerate()}Hz | {wf.getnframes()} frames")
        
        t_next = time.perf_counter()
        
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
                pcm_bytes = sel.tobytes(order="C")
                send_block_udp(sock, seq, pcm_bytes)
                seq = (seq + 1) & 0xFFFFFFFF

                t_next += block_duration_s
                sleep_time = t_next - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: behind by {-sleep_time*1000:.2f}ms")
                    
            print("EOF: Replaying...")

    sock.close()

def main():
    global running
    global UDP_IP, UDP_PORT

    args = arg_parsing()
    cols = [m - 1 for m in args.mics]
    UDP_IP   = args.ip
    UDP_PORT = args.port

    print("Available devices:")
    devices = sd.query_devices()
    print(devices if devices else "N/A")

    if (args.mode == 1): #record live audio and send it and save it to a wav

        ethernet_thread = threading.Thread(target=ethernet_worker, args=(cols, args.gain), daemon=True)
        wav_thread = threading.Thread(target=wav_worker, args=(SAVE_WAVE,), daemon=True)
        wav_thread.start()
        ethernet_thread.start()

        try:
            with sd.InputStream(device=args.device, channels=args.channels, samplerate=RATE, blocksize=FRAMES_PER_BLOCK, dtype=SAMPLE_DTYPE, callback=audio_callback,):
                print("Streaming. Press Ctrl+C to stop.")
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    running = False
                    print("Stopping...")
                    ethernet_thread.join(timeout=2)
                    wav_thread.join(timeout=2)

        except Exception as e:
            print("Audio device unavailable:", e)
            running = False
            ethernet_thread.join(timeout=2)
            wav_thread.join(timeout=2)
    else: #Playback version
        playback_loop(cols, args.gain, args.open_wav)


if __name__ == "__main__":
    main()
