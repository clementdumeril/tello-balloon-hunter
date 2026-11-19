"""Low-latency reader for the Tello H.264 video (replaces djitellopy's BackgroundFrameRead).

djitellopy opens the UDP stream with FFmpeg's defaults, made for smooth playback: input buffering, a long stream
analysis at startup, frame-threaded decoding (each decoder thread holds frames back) and no read timeout.
Here:
  * format options: fflags=nobuffer (no input buffer), small probesize / analyzeduration, flags=low_delay;
  * decoder: low_delay flag; never frame threading (it holds images back: +260 ms measured);
  * only the most recent decoded image is kept (the consumer never reads a stale one);
  * a 1 s read timeout: after a Wi-Fi drop the container is closed (frees UDP port 11111) and reopened by the
    reader itself; no 'Address already in use', no hanging thread.
  * never a growing delay (Nov 12: the video in RViz was ONE MINUTE late): when the decoder is slower than the
    30 images/s the Tello sends (CPU busy: RViz, ArUco...), the packets used to pile up in FFmpeg's UDP buffer
    (fifo_size was 50 M x 188 bytes, i.e. unlimited) and the delay grew ~27 s per minute. Now: the buffer holds
    ~1.5 s, and when the reader sees it is behind (it never has to wait for data any more) it drops packets up
    to the next key frame and starts again from the newest images;
  * the RGB conversion is only done for the image the consumer reads (not for every decoded one).
Same interface as djitellopy's reader: .frame (RGB numpy array), .stop(). Extra: .stats() (decode rate, decode
time, reconnections, skipped packets) and .frame_time (when the current image was decoded).
"""
import collections
import threading
import time

import av
import numpy as np

# fifo_size is in 188-byte units: 8000 = 1.5 MB, ~1.5-3 s of Tello video (2-4 Mbit/s); a fuller buffer drops data
# (overrun_nonfatal) instead of delaying every image after it
URL = "udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=8000"
BEHIND_WINDOW = 1.0     # s of packets looked at to decide that the reader is behind
BEHIND_WAIT = 0.03      # behind if it waited for data less than 3 % of that time (real time: most of it)
MAX_SKIP = 2.0          # s: give up waiting for a key frame and decode again (artifacts until the next one)
# chosen with a latency bench on a fake H.264 stream: nobuffer + short probe + decoder low_delay gave the best tails
# (p90 49 ms vs 60, max 68 vs 120 for djitellopy). Adding max_delay=0 / flags=low_delay at the FORMAT level lost
# images, and FRAME threading (thread_type AUTO) added ~260 ms: never enable it.
FORMAT_OPTIONS = {"fflags": "nobuffer", "probesize": "32768", "analyzeduration": "0"}


class LowLatencyReader:
    def __init__(self, url=URL, low_latency=True, log=None, format_options=None, codec_low_delay=None,
                 thread_type=None):
        """low_latency=True: the settings chosen from the latency bench; the other
        arguments override single settings for experiments."""
        self.url = url
        self.low_latency = low_latency
        self.format_options = FORMAT_OPTIONS if format_options is None and low_latency else (format_options or {})
        self.codec_low_delay = low_latency if codec_low_delay is None else codec_low_delay
        self.thread_type = thread_type
        self.log = log or (lambda msg: None)
        self._lock = threading.Lock()
        self._frame = np.zeros((300, 400, 3), np.uint8)  # same placeholder as djitellopy (skipped by the driver)
        self._av_frame = None   # newest decoded frame, converted to RGB only when read
        self.frame_time = 0.0
        self.skipped = 0        # packets dropped to catch up
        self.catch_ups = 0
        self.stopped = False
        self.container = None
        self.n_frames = 0
        self.reconnects = 0
        self.decode_s = 0.0
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    @property
    def frame(self):
        """Newest image (RGB). The same array object until a newer image is decoded (the driver relies on it)."""
        with self._lock:
            f, self._av_frame = self._av_frame, None
        if f is not None:
            img = f.to_ndarray(format="rgb24")
            with self._lock:
                if self._av_frame is None:  # no newer one decoded meanwhile
                    self._frame = img
        with self._lock:
            return self._frame

    def stop(self, wait=2.0):
        """Only the reader thread touches the FFmpeg container (closing it from another thread while it decodes
        crashes the process): raise the flag, the thread closes it within the 1 s read timeout."""
        self.stopped = True
        if wait and threading.current_thread() is not self.worker:
            self.worker.join(wait)

    def stats(self):
        n = max(self.n_frames, 1)
        return {"frames": self.n_frames, "decode_ms": 1000 * self.decode_s / n, "reconnects": self.reconnects,
                "skipped_packets": self.skipped, "catch_ups": self.catch_ups}

    def _close(self):
        c, self.container = self.container, None
        if c is not None:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass

    def _open(self):
        c = av.open(self.url, options=dict(self.format_options), timeout=(10.0, 1.0))
        stream = c.streams.video[0]
        ctx = stream.codec_context
        if self.codec_low_delay:
            try:
                ctx.flags |= av.codec.context.Flags.low_delay
            except Exception:  # noqa: BLE001
                pass
        if self.thread_type:
            try:
                ctx.thread_type = self.thread_type
            except Exception:  # noqa: BLE001
                pass
        return c, stream

    def _decode(self, packet):
        return packet.decode()

    def _run(self):
        while not self.stopped:
            try:
                self.container, stream = self._open()
                hist = collections.deque()  # (t, waited for this packet) over the last BEHIND_WINDOW s
                skip_since = None
                t_end = time.perf_counter()
                for packet in self.container.demux(stream):
                    if self.stopped:
                        break
                    t0 = time.perf_counter()
                    hist.append((t0, t0 - t_end))  # time spent waiting for this packet to arrive
                    while hist and hist[0][0] < t0 - BEHIND_WINDOW:
                        hist.popleft()
                    if skip_since is None and len(hist) > 10 and t0 - hist[0][0] > 0.8 * BEHIND_WINDOW \
                            and sum(w for _, w in hist) < BEHIND_WAIT * BEHIND_WINDOW:
                        # never waiting for data: packets are queued, the images are late -> catch up
                        skip_since = t0
                        self.catch_ups += 1
                        self.log("video: decoding slower than the drone sends, dropping late images to catch up")
                    if skip_since is not None:
                        if packet.is_keyframe or t0 - skip_since > MAX_SKIP:
                            skip_since = None
                            hist.clear()
                        else:
                            self.skipped += 1
                            t_end = time.perf_counter()
                            continue
                    for fr in self._decode(packet):
                        with self._lock:
                            self._av_frame = fr
                            self.frame_time = time.time()
                        self.n_frames += 1
                    t_end = time.perf_counter()
                    self.decode_s += t_end - t0
            except Exception as e:  # noqa: BLE001  (timeout after a Wi-Fi drop, corrupted stream...)
                if self.stopped:
                    break
                self.log(f"video: {e.__class__.__name__}, reopening the stream")
            self._close()
            if not self.stopped:
                self.reconnects += 1
                time.sleep(0.3)
