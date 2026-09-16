"""V4L2 control access that works on the host AND inside the container.

WHY THIS IS NOT JUST subprocess('v4l2-ctl'). Auto-exposure has to write exposure
and gain continuously, and vslam-min:3.2 does NOT ship v4l-utils -- the drone
image installed it, this one deliberately does not. That is why every runner so
far set exposure from the HOST before starting the container and then left it
fixed at 1500/800 for the whole run. Fixed exposure is exactly what auto-exposure
exists to replace, so as long as writes need v4l2-ctl, AE and cuVSLAM cannot be
in the same process.

So: try v4l2-ctl, and fall back to the VIDIOC_S_CTRL ioctl directly. Control IDs
are DISCOVERED BY NAME through VIDIOC_QUERYCTRL rather than hardcoded, because
these are Arducam custom controls whose IDs are not in the kernel headers and
guessing them would write to whatever else happens to live at that offset.

ONE HARDWARE QUIRK THIS MODULE KNOWS ABOUT. analogue_gain advertises max=1200.
Above roughly 1100 the write is ACCEPTED, v4l2-ctl reads the new value back, and
the image does not get any brighter. So the advertised maximum is not the usable
maximum, and a controller that trusts it will sit at the top of its range
believing it has authority it does not have.

STATUS: the v4l2-ctl path is what every measurement to date used. THE IOCTL
FALLBACK HAS NOT BEEN RUN ON HARDWARE -- it was written while jetson9 was powered
down. Until it is, treat a container-side AE run as unproven, and check
verify() before believing a write landed.
"""
import ctypes
import errno
import fcntl
import struct
import subprocess

GAIN_USABLE_MAX = 1100          # not the advertised 1200; see the module docstring

_IOC_WRITE, _IOC_READ = 1, 2
_QUERYCTRL_SIZE = 68            # sizeof(struct v4l2_queryctrl)
_CONTROL_SIZE = 8               # sizeof(struct v4l2_control)
_NEXT_CTRL = 0x80000000


def _iowr(nr, size):
    req = ((_IOC_WRITE | _IOC_READ) << 30) | (size << 16) | (0x56 << 8) | nr
    # fcntl.ioctl wants a value that fits a C int, and these all have the top bit
    # set, so reinterpret as signed rather than letting it raise OverflowError.
    return struct.unpack('i', struct.pack('I', req))[0]


VIDIOC_QUERYCTRL = _iowr(36, _QUERYCTRL_SIZE)
VIDIOC_G_CTRL = _iowr(27, _CONTROL_SIZE)
VIDIOC_S_CTRL = _iowr(28, _CONTROL_SIZE)


RAW_NAMES = {}          # normalised -> the label the ioctl reported


def have_v4l2_ctl():
    try:
        subprocess.run(['v4l2-ctl', '--version'], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)
        return True
    except (FileNotFoundError, OSError):
        return False


def normalise(name):
    """VIDIOC_QUERYCTRL name -> the id v4l2-ctl would use.

    The two interfaces disagree and it is not obvious from either side. The ioctl
    reports human-readable labels -- 'Exposure', 'Analogue Gain' -- while
    v4l2-ctl -c takes lowercase snake_case ids, 'exposure', 'analogue_gain'.
    Written against the label, every lookup fails with the control sitting right
    there in the error message. Some Arducam controls are already snake_case
    ('frame_rate', 'trigger_mode'), so normalising both sides is what works.
    """
    out = []
    for ch in name.strip().lower():
        out.append(ch if ch.isalnum() else '_')
    s = ''.join(out)
    while '__' in s:
        s = s.replace('__', '_')
    return s.strip('_')


def enumerate_controls(device='/dev/video0'):
    """{normalised_name: (id, minimum, maximum, default)} via VIDIOC_QUERYCTRL.

    Keys are normalised so a caller can ask for 'exposure' whichever backend is
    in use. The raw label is kept as .raw_names for diagnostics.
    """
    out = {}
    with open(device, 'rb', buffering=0) as fh:
        qid = _NEXT_CTRL
        while True:
            buf = bytearray(_QUERYCTRL_SIZE)
            struct.pack_into('I', buf, 0, qid)
            try:
                fcntl.ioctl(fh, VIDIOC_QUERYCTRL, buf, True)
            except OSError as e:
                if e.errno in (errno.EINVAL, errno.ENOTTY):
                    break
                raise
            cid, ctype = struct.unpack_from('II', buf, 0)
            name = bytes(buf[8:40]).split(b'\0')[0].decode('ascii', 'replace')
            lo, hi, _step, dflt = struct.unpack_from('iiii', buf, 40)
            if cid == 0 or cid == qid & ~_NEXT_CTRL:
                break
            out[normalise(name)] = (cid, lo, hi, dflt)
            RAW_NAMES[normalise(name)] = name
            qid = cid | _NEXT_CTRL
    return out


class Controls:
    """Set and read camera controls, whichever mechanism is available."""

    def __init__(self, device='/dev/video0', prefer_ioctl=False):
        self.device = device
        self.use_ctl = (not prefer_ioctl) and have_v4l2_ctl()
        self._ids = None
        self.writes = 0

    @property
    def backend(self):
        return 'v4l2-ctl' if self.use_ctl else 'ioctl'

    def _id_of(self, name):
        if self._ids is None:
            self._ids = enumerate_controls(self.device)
        key = normalise(name)
        if key not in self._ids:
            raise KeyError('%s has no control named %r (normalised %r); available: %s'
                           % (self.device, name, key, ', '.join(sorted(self._ids))))
        return self._ids[key][0]

    def set(self, name, value):
        value = int(value)
        if name == 'analogue_gain' and value > GAIN_USABLE_MAX:
            value = GAIN_USABLE_MAX
        self.writes += 1
        if self.use_ctl:
            r = subprocess.run(['v4l2-ctl', '-d', self.device, '-c',
                                '%s=%d' % (name, value)],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE)
            if r.returncode == 0:
                return True
            # Fall through to the ioctl path rather than failing: a v4l2-ctl that
            # exists but cannot drive this control is no better than none.
            self.use_ctl = False
        buf = bytearray(struct.pack('Ii', self._id_of(name), value))
        with open(self.device, 'rb', buffering=0) as fh:
            fcntl.ioctl(fh, VIDIOC_S_CTRL, buf, True)
        return True

    def get(self, name):
        buf = bytearray(struct.pack('Ii', self._id_of(name), 0))
        with open(self.device, 'rb', buffering=0) as fh:
            fcntl.ioctl(fh, VIDIOC_G_CTRL, buf, True)
        return struct.unpack('Ii', bytes(buf))[1]

    def verify(self, name, value):
        """Write, read back, and say whether it actually took.

        Worth using on gain specifically: above the usable ceiling the write is
        accepted and reads back correctly while doing nothing, so agreement here
        is necessary but not sufficient -- only the image tells you the rest.
        """
        self.set(name, value)
        try:
            got = self.get(name)
        except (OSError, KeyError):
            return None
        return got == int(value)
