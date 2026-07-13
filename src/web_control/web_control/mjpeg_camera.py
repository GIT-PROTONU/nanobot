"""Zero-dependency V4L2 MJPEG capture for a UVC webcam.

Grabs the camera's hardware-encoded MJPEG frames (no decode / re-encode) via the
V4L2 MMAP streaming API and shares the latest frame with HTTP clients, so
web_control can serve a low-latency /stream.mjpg without OpenCV / ffmpeg / ROS
image topics — it's just memcpy of JPEG bytes, so CPU and memory stay tiny.

Struct layouts and ioctl sizes are validated for aarch64 (LP64): the v4l2_format
union is 8-byte aligned (v4l2_window has pointers) so `type` is padded → 208 bytes;
v4l2_buffer is 88 bytes (timeval/timecode + the 8-byte `m` union).
"""
import ctypes
import fcntl
import glob
import mmap
import os
import select
import struct
import threading
import time

u32 = ctypes.c_uint32


def _IOC(d, nr, sz): return (d << 30) | (sz << 16) | (ord('V') << 8) | nr
def _IOR(nr, sz): return _IOC(2, nr, sz)
def _IOW(nr, sz): return _IOC(1, nr, sz)
def _IOWR(nr, sz): return _IOC(3, nr, sz)
def _fourcc(s): return s[0] | s[1] << 8 | s[2] << 16 | s[3] << 24


class _Pix(ctypes.Structure):
    _fields_ = [("width", u32), ("height", u32), ("pixelformat", u32), ("field", u32),
                ("bytesperline", u32), ("sizeimage", u32), ("colorspace", u32), ("priv", u32),
                ("flags", u32), ("enc", u32), ("quantization", u32), ("xfer_func", u32)]


class _Format(ctypes.Structure):          # union is 8-aligned -> pad after type, total 208
    _fields_ = [("type", u32), ("_pad", u32), ("pix", _Pix),
                ("raw", ctypes.c_uint8 * (200 - ctypes.sizeof(_Pix)))]


class _ReqBufs(ctypes.Structure):
    _fields_ = [("count", u32), ("type", u32), ("memory", u32), ("capabilities", u32),
                ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [("type", u32), ("flags", u32), ("frames", ctypes.c_uint8),
                ("seconds", ctypes.c_uint8), ("minutes", ctypes.c_uint8),
                ("hours", ctypes.c_uint8), ("userbits", ctypes.c_uint8 * 4)]


class _BufM(ctypes.Union):
    _fields_ = [("offset", u32), ("userptr", ctypes.c_ulong), ("fd", ctypes.c_int32)]


class _Buffer(ctypes.Structure):
    _fields_ = [("index", u32), ("type", u32), ("bytesused", u32), ("flags", u32),
                ("field", u32), ("timestamp", _Timeval), ("timecode", _Timecode),
                ("sequence", u32), ("memory", u32), ("m", _BufM), ("length", u32),
                ("reserved2", u32), ("request_fd", ctypes.c_int32)]


class _Cap(ctypes.Structure):
    _fields_ = [("driver", ctypes.c_char * 16), ("card", ctypes.c_char * 32),
                ("bus_info", ctypes.c_char * 32), ("version", u32),
                ("capabilities", u32), ("device_caps", u32), ("reserved", u32 * 3)]


class _Fract(ctypes.Structure):
    _fields_ = [("numerator", u32), ("denominator", u32)]


class _CaptureParm(ctypes.Structure):
    _fields_ = [("capability", u32), ("capturemode", u32), ("timeperframe", _Fract),
                ("extendedmode", u32), ("readbuffers", u32), ("reserved", u32 * 4)]


class _StreamParm(ctypes.Structure):
    _fields_ = [("type", u32), ("capture", _CaptureParm),
                ("raw", ctypes.c_uint8 * (200 - ctypes.sizeof(_CaptureParm)))]


_QUERYCAP = _IOR(0, ctypes.sizeof(_Cap))
_S_FMT = _IOWR(5, ctypes.sizeof(_Format))
_REQBUFS = _IOWR(8, ctypes.sizeof(_ReqBufs))
_QUERYBUF = _IOWR(9, ctypes.sizeof(_Buffer))
_QBUF = _IOWR(15, ctypes.sizeof(_Buffer))
_DQBUF = _IOWR(17, ctypes.sizeof(_Buffer))
_STREAMON = _IOW(18, 4)
_STREAMOFF = _IOW(19, 4)
_S_PARM = _IOWR(22, ctypes.sizeof(_StreamParm))

_CAP_VIDEO_CAPTURE = 0x00000001
_BUF_TYPE = 1          # V4L2_BUF_TYPE_VIDEO_CAPTURE
_MEMORY_MMAP = 1
_FIELD_NONE = 1
_MJPG = _fourcc(b'MJPG')
FOURCC_YUYV = _fourcc(b'YUYV')          # public: gpu_vision.py requests this format


def find_camera():
    """First /dev/videoN that is a UVC capture device (skips SoC codec/di nodes)."""
    for dev in sorted(glob.glob("/dev/video*")):
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            cap = _Cap()
            fcntl.ioctl(fd, _QUERYCAP, cap)
            if (cap.device_caps & _CAP_VIDEO_CAPTURE) and cap.driver == b"uvcvideo":
                return dev
        except OSError:
            pass
        finally:
            os.close(fd)
    return None


class MjpegCamera:
    """Low-level V4L2 MMAP capture. Call read() for the next frame's raw bytes.
    Despite the name, `fourcc` lets a caller request any format the camera supports
    (e.g. raw YUYV for the GPU vision path in gpu_vision.py) — defaults to MJPG so
    every existing caller (the browser stream) is unaffected."""

    def __init__(self, dev, width=640, height=480, fps=15, nbufs=4, fourcc=_MJPG):
        self.fd = os.open(dev, os.O_RDWR)
        self._streaming = False
        self.maps = []
        try:
            f = _Format(type=_BUF_TYPE)
            f.pix.width, f.pix.height = width, height
            f.pix.pixelformat, f.pix.field = fourcc, _FIELD_NONE
            fcntl.ioctl(self.fd, _S_FMT, f)
            if f.pix.pixelformat != fourcc:
                raise OSError(f"camera did not accept requested format {fourcc!r}")
            self.width, self.height = f.pix.width, f.pix.height
            self.bytesperline = f.pix.bytesperline
            # best-effort frame-rate cap (lower CPU/bandwidth); ignore if unsupported
            try:
                sp = _StreamParm(type=_BUF_TYPE)
                sp.capture.timeperframe.numerator = 1
                sp.capture.timeperframe.denominator = max(1, int(fps))
                fcntl.ioctl(self.fd, _S_PARM, sp)
            except OSError:
                pass
            r = _ReqBufs(count=nbufs, type=_BUF_TYPE, memory=_MEMORY_MMAP)
            fcntl.ioctl(self.fd, _REQBUFS, r)
            for i in range(r.count):
                b = _Buffer(index=i, type=_BUF_TYPE, memory=_MEMORY_MMAP)
                fcntl.ioctl(self.fd, _QUERYBUF, b)
                self.maps.append(mmap.mmap(self.fd, b.length, mmap.MAP_SHARED,
                                           mmap.PROT_READ, offset=b.m.offset))
                fcntl.ioctl(self.fd, _QBUF, b)
            fcntl.ioctl(self.fd, _STREAMON, struct.pack("I", _BUF_TYPE))
            self._streaming = True
            self._poll = select.poll()
            self._poll.register(self.fd, select.POLLIN)
        except Exception:
            self.close()
            raise

    def read(self, timeout_ms=1000):
        # Wait for a frame via poll() — that RELEASES the GIL, so the HTTP threads
        # keep running. A blocking DQBUF would hold the GIL the whole wait and
        # starve them (caps the stream to a fraction of the capture rate).
        if not self._poll.poll(timeout_ms):
            return None                              # no frame within timeout
        b = _Buffer(type=_BUF_TYPE, memory=_MEMORY_MMAP)
        fcntl.ioctl(self.fd, _DQBUF, b)              # ready now -> returns at once
        jpeg = self.maps[b.index][:b.bytesused]      # copy out of the mmap buffer
        fcntl.ioctl(self.fd, _QBUF, b)
        return jpeg

    def close(self):
        try:
            if self._streaming:
                fcntl.ioctl(self.fd, _STREAMOFF, struct.pack("I", _BUF_TYPE))
        except OSError:
            pass
        for mm in self.maps:
            try:
                mm.close()
            except Exception:
                pass
        try:
            os.close(self.fd)
        except OSError:
            pass


class CameraStream:
    """Shares one camera among HTTP clients. Opens the device on the first viewer
    and closes it when the last one leaves, so it costs nothing when nobody is
    watching. Each client is served the latest frame (slow clients skip frames)."""

    def __init__(self, dev=None, width=640, height=480, fps=15, logger=None):
        self._cfg = dict(width=width, height=height, fps=fps)
        self._dev = dev or None
        self._log = logger or (lambda *_: None)
        self._cond = threading.Condition()
        self._viewers = 0
        self._thread = None
        self._frame = None
        self._seq = 0
        self._run = False

    def _loop(self):
        try:
            dev = self._dev or find_camera()
            if not dev:
                raise OSError("no UVC camera found")
            cam = MjpegCamera(dev, **self._cfg)
            self._log(f"camera streaming {dev} {cam.width}x{cam.height}")
        except Exception as exc:
            self._log(f"camera open failed: {exc}")
            with self._cond:
                self._run = False
                self._cond.notify_all()
            return
        period = 1.0 / max(1, self._cfg.get("fps", 15))
        next_t = time.monotonic()
        while self._run:
            try:
                jpeg = cam.read(1000)          # poll+DQBUF (GIL released in poll)
                if jpeg is None:
                    continue
                while True:                    # drain queued frames -> lowest latency
                    extra = cam.read(0)
                    if extra is None:
                        break
                    jpeg = extra
            except Exception as exc:
                if self._run:
                    self._log(f"camera read error: {exc}")
                break
            with self._cond:
                self._frame = jpeg
                self._seq += 1
                self._cond.notify_all()
            # Pace to the target fps. The sleep both caps CPU/bandwidth and, crucially,
            # releases the GIL so the HTTP sender threads actually get to run (at the
            # camera's native rate the buffers are always ready, so poll() never blocks).
            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()
        cam.close()
        with self._cond:
            self._run = False
            self._cond.notify_all()

    def add_viewer(self):
        with self._cond:
            self._viewers += 1
            if self._thread is None or not self._thread.is_alive():
                self._run = True
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()

    def remove_viewer(self):
        with self._cond:
            self._viewers -= 1
            if self._viewers <= 0:
                self._viewers = 0
                self._run = False
                self._cond.notify_all()

    def running(self):
        with self._cond:
            return self._run

    def get_frame(self, last_seq, timeout=5.0):
        """Block until a frame newer than last_seq; return (seq, jpeg|None)."""
        with self._cond:
            if not self._cond.wait_for(
                    lambda: self._seq != last_seq or not self._run, timeout):
                return last_seq, None
            return self._seq, self._frame
