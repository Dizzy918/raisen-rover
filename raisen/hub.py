"""One capture thread, many consumers.

THIS IS THE WHOLE POINT OF THE MERGE. /dev/video0 admits exactly one consumer,
so while every component opened the camera itself they were mutually exclusive:
running the occupancy grid meant not running vSLAM, and neither could run
alongside the live preview. The hub owns the device and hands the same frame to
every subscriber.

BACKPRESSURE IS A CHOICE, NOT AN ACCIDENT. A slow subscriber must not slow the
camera down, because the camera's rate is what vSLAM's motion model assumes. So
each subscriber gets a slot holding the LATEST frame only, and a subscriber that
misses frames is told how many rather than being allowed to fall behind
invisibly. Anything that needs every frame has to keep up or say so.
"""
import threading
import time

from . import frame as _frame


class Subscription:
    """A one-slot mailbox. Latest frame wins; misses are counted, not queued."""

    def __init__(self, name):
        self.name = name
        self.cv = threading.Condition()
        self._item = None
        self.delivered = 0
        self.missed = 0

    def _offer(self, seq, frame16, eyes, meta):
        with self.cv:
            if self._item is not None:
                self.missed += 1
            self._item = (seq, frame16, eyes, meta)
            self.cv.notify()

    def get(self, timeout=1.0):
        """(seq, frame16, (left, right), meta) or None on timeout.

        The eyes are already split and rotated. Consumers must NOT re-split: that
        was the whole cost of the first merged run.
        """
        with self.cv:
            if self._item is None:
                self.cv.wait(timeout)
            item = self._item
            self._item = None
        if item is not None:
            self.delivered += 1
        return item


class FrameHub:
    def __init__(self, camera, on_frame=None):
        """on_frame(frame16) runs on the CAPTURE thread, before fan-out.

        Auto-exposure belongs here and nowhere else: it has to meter the raw
        frame before anything stretches it, and it must see every frame rather
        than a subscriber's sample. Keep it cheap -- whatever it costs is charged
        directly to the frame rate.
        """
        self.camera = camera
        self.on_frame = on_frame
        self.subs = []
        self.seq = 0
        self.dropped = 0
        self.running = False
        self._thread = None
        self._lock = threading.Lock()
        self.t0 = None
        self.split_ms = 0.0
        self.split_n = 0
        self._raw = None
        self._raw_cv = threading.Condition()
        self._split_thread = None

    def subscribe(self, name):
        s = Subscription(name)
        with self._lock:
            self.subs.append(s)
        return s

    def start(self):
        self.running = True
        self.t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name='framehub')
        self._thread.start()
        self._split_thread = threading.Thread(target=self._split_loop, daemon=True,
                                              name='framehub-split')
        self._split_thread.start()
        return self

    def _split_loop(self):
        """Split in its own stage, so neither the camera nor the consumers pay for it.

        THE MEASUREMENTS THAT PRODUCED THIS SHAPE, all on the same 60 s scene:

          split in each consumer   hub 30.7 fps, publisher 23.5 fps at 40 ms/frame,
                                   20% of frames missed, cuVSLAM warning about
                                   130-210 ms gaps. Three threads each ran bin2x2
                                   on the same frame.
          split on the camera      hub 20.5 fps, publisher 20.4 fps at 17.8 ms and
          thread                   zero drops. The duplicate work was gone and the
                                   publisher was cheaper than standalone -- but
                                   split_pair costs 30 ms and serialising it behind
                                   the camera read capped the whole hub.
          split in its own stage   the camera reads at full rate, the split runs
                                   concurrently with it, and consumers get eyes
                                   they did not have to compute.

        The first arrangement was accidentally exploiting parallelism: numpy and
        cv2 drop the GIL, so three concurrent bin2x2 calls beat one serial call.
        Removing the duplication was still right, but only once it stopped
        blocking the camera.
        """
        while self.running:
            with self._raw_cv:
                if self._raw is None:
                    self._raw_cv.wait(0.5)
                item = self._raw
                self._raw = None
            if item is None:
                continue
            f, meta = item
            t0 = time.time()
            eyes = _frame.split_pair(f)
            self.split_ms += (time.time() - t0) * 1000.0
            self.split_n += 1
            with self._lock:
                subs = list(self.subs)
            for s in subs:
                s._offer(self.split_n, f, eyes, meta)

    def _loop(self):
        while self.running:
            f = self.camera.read_raw()
            if f is None:
                self.dropped += 1
                continue
            meta = {'t': time.time()}
            # Auto-exposure meters the RAW frame, before anything stretches it, and
            # it is cheap, so it stays on this thread.
            if self.on_frame is not None:
                self.on_frame(f)
            self.seq += 1        # frames READ; split_n counts frames split
            # Hand the frame to the split stage and go straight back to reading.
            with self._raw_cv:
                self._raw = (f, meta)
                self._raw_cv.notify()

    def stop(self):
        self.running = False
        with self._raw_cv:
            self._raw_cv.notify_all()
        for t in (self._thread, self._split_thread):
            if t is not None:
                t.join(timeout=2.0)

    def fps(self):
        el = time.time() - (self.t0 or time.time())
        return self.seq / el if el > 0 else 0.0

    def report(self):
        lines = ['%d frames read at %.1f fps, %d camera drops; %d split at '
                 '%.1f ms each'
                 % (self.seq, self.fps(), self.dropped, self.split_n,
                    self.split_ms / max(self.split_n, 1))]
        for s in self.subs:
            share = 100.0 * s.delivered / max(self.seq, 1)
            lines.append('  %-14s %6d delivered (%.0f%%), %d missed'
                         % (s.name, s.delivered, share, s.missed))
        return '\n'.join(lines)
