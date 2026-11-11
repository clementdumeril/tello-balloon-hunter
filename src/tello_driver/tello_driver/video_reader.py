"""Low-latency reader for the Tello H.264 video (replaces djitellopy's BackgroundFrameRead).

djitellopy opens the UDP stream with FFmpeg's defaults, made for smooth playback: input buffering, a long stream
analysis at startup, frame-threaded decoding (each decoder thread holds frames back) and no read timeout.
Here:
  * format options: fflags=nobuffer (no input buffer), small probesize / analyzeduration, flags=low_delay;
  * decoder: low_delay flag; never frame threading (it holds images back: +260 ms measured);
  * only the most recent decoded image is kept (the consumer never reads a stale one);
  * a 1 s read timeout: after a Wi-Fi drop the container is closed (frees UDP port 11111) and reopened by the
    reader itself; no 'Address already in use', no hanging thread.
Same interface as djitellopy's reader: .frame (RGB numpy array), .stop(). Extra: .stats() (decode rate, decode
time, reconnections) and .frame_time (when the current image was decoded).
"""
import threading
import time

import av
import numpy as np

URL = "udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"
# chosen with tools/video_latency_bench.py: nobuffer + short probe + decoder low_delay gave the best tails
# (p90 49 ms vs 60, max 68 vs 120 for djitellopy). Adding max_delay=0 / flags=low_delay at the FORMAT level lost
# images, and FRAME threading (thread_type AUTO) added ~260 ms: never enable it.
FORMAT_OPTIONS = {"fflags": "nobuffer", "probesize": "32768", "analyzeduration": "0"}


class LowLatencyReader:
    def __init__(self, url=URL, low_latency=True, log=None, format_options=None, codec_low_delay=None,
                 thread_type=None):
        """low_latency=True: the settings chosen from the bench (tools/video_latency_bench.py); the other
        arguments override single settings for experiments."""
        self.url = url
        self.low_latency = low_latency
        self.format_options = FORMAT_OPTIONS if format_options is None and low_latency else (format_options or {})
        self.codec_low_delay = low_latency if codec_low_delay is None else codec_low_delay
        self.thread_type = thread_type
        self.log = log or (lambda msg: None)
        self._lock = threading.Lock()
        self._frame = np.zeros((300, 400, 3), np.uint8)  # same placeholder as djitellopy (skipped by the driver)
        self.frame_time = 0.0
        self.stopped = False
        self.container = None
        self.n_frames = 0
        self.reconnects = 0
        self.decode_s = 0.0
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    @property
    def frame(self):
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
        return {"frames": self.n_frames, "decode_ms": 1000 * self.decode_s / n, "reconnects": self.reconnects}

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

    def _run(self):
        while not self.stopped:
            try:
                self.container, stream = self._open()
                for packet in self.container.demux(stream):
                    if self.stopped:
                        break
                    t0 = time.perf_counter()
                    for fr in packet.decode():
                        img = fr.to_ndarray(format="rgb24")
                        with self._lock:
                            self._frame = img
                            self.frame_time = time.time()
                        self.n_frames += 1
                    self.decode_s += time.perf_counter() - t0
            except Exception as e:  # noqa: BLE001  (timeout after a Wi-Fi drop, corrupted stream...)
                if self.stopped:
                    break
                self.log(f"video: {e.__class__.__name__}, reopening the stream")
            self._close()
            if not self.stopped:
                self.reconnects += 1
                time.sleep(0.3)
