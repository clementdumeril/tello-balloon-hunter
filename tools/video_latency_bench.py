#!/usr/bin/env python3
"""Video reader latency bench (no drone needed): a fake Tello stream -> each reader -> latency per image.

    python3 tools/video_latency_bench.py          (Ubuntu / WSL, ROS not needed; PyAV with libx264)

An H.264 encoder (x264, zerolatency, baseline, 960x720 at 30 fps, like the Tello) sends images over UDP on
localhost; each image carries its number as black/white blocks. Every reader variant decodes the same kind of
stream and the delay between sending and the decoded image is measured. Ports 11300+ (not the Tello's 11111).
"""
import sys, time, socket, threading, numpy as np, av
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "tello_driver"))
from tello_driver.video_reader import LowLatencyReader
PORT = [11211]; W, H, FPS, NBITS = 960, 720, 30, 16
sent = {}
def image(i):
    img = np.full((H, W, 3), 120, np.uint8)
    img[200:, :] = (np.random.default_rng(i).integers(0, 255, (H - 200, W, 3))).astype(np.uint8) // 4 + 90  # texture
    for b in range(NBITS):
        img[20:100, 20 + b * 56: 20 + b * 56 + 50] = 255 if (i >> b) & 1 else 0
    return img
def index(img):
    return sum((1 << b) for b in range(NBITS) if img[60, 45 + b * 56].mean() > 128)
def sender(seconds):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    enc = av.CodecContext.create("libx264", "w"); enc.width, enc.height, enc.pix_fmt = W, H, "yuv420p"
    from fractions import Fraction; enc.time_base = Fraction(1, FPS); enc.framerate = Fraction(FPS, 1); enc.gop_size = 30
    enc.options = {"preset": "ultrafast", "tune": "zerolatency", "profile": "baseline"}
    t0 = time.time()
    for i in range(int(seconds * FPS)):
        while time.time() < t0 + i / FPS: time.sleep(0.001)
        fr = av.VideoFrame.from_ndarray(image(i), format="rgb24").reformat(format="yuv420p"); fr.pts = i
        sent[i] = time.time()
        for pkt in enc.encode(fr):
            b = bytes(pkt)
            for k in range(0, len(b), 1400): s.sendto(b[k:k + 1400], ("127.0.0.1", PORT[0]))
def measure(make_reader, label, seconds=12):
    sent.clear()
    r = make_reader()
    th = threading.Thread(target=sender, args=(seconds,), daemon=True); th.start()
    lat, last, t_end = [], None, time.time() + seconds + 1
    while time.time() < t_end:
        f = r.frame
        if f is not None and f is not last and f.shape[0] == H:
            last = f; i = index(f)
            if i in sent and time.time() - sent[i] < 5: lat.append((i, time.time() - sent[i]))
        time.sleep(0.003)
    th.join(); r.stop()
    L = np.array([l for i, l in lat if i > FPS * 2]) * 1000  # skip the first 2 s (startup)
    if len(L) == 0:
        f = r.frame; print(label, "NO FRAMES matched; seen", len(lat), "last shape", None if f is None else f.shape,
              "stats", getattr(r, "stats", lambda: "")()); return
    n_frames = len({i for i, _ in lat})
    print(f"{label:34s} latency median {np.median(L):6.1f} ms  p90 {np.percentile(L, 90):6.1f} ms  max {L.max():6.1f} ms"
          f"  | frames seen {n_frames}/{int(seconds * FPS)}")
    time.sleep(1.0)
def djitellopy_reader():
    from djitellopy.tello import BackgroundFrameRead
    r = BackgroundFrameRead(None, f"udp://@0.0.0.0:{PORT[0]}"); r.start(); return r
def url(): return f"udp://@0.0.0.0:{PORT[0]}?overrun_nonfatal=1&fifo_size=50000000"
def start_dj():
    holder = {}
    t = threading.Thread(target=lambda: holder.setdefault("r", djitellopy_reader()), daemon=True); t.start()
    time.sleep(0.2)
    class Proxy:
        @property
        def frame(self): return holder["r"].frame if "r" in holder else None
        def stop(self):
            if "r" in holder:
                holder["r"].stop()
    return Proxy()
VARIANTS = [
    ("djitellopy (today)", None),
    ("new reader (driver default)", dict(low_latency=True)),
    ("new reader, FFmpeg defaults", dict(low_latency=False)),
    ("new reader + FRAME threads", dict(low_latency=True, thread_type="AUTO")),
] * 2
for k, (label, kw) in enumerate(VARIANTS):
    PORT[0] = 11300 + k
    measure(start_dj if kw is None else (lambda kw=kw: LowLatencyReader(url(), **kw)), label, 10)
import os; os._exit(0)
