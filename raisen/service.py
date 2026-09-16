"""Taking /dev/video0 away from narsil-vision, and reliably giving it back.

Copy-pasted into twenty files before this. Both of its failure modes cost real
time, so they are written down here once:

  RESTART POLICY. narsil-vision runs with restart=on-failure and unlimited
  retries, and `docker stop` exits 137, which counts as a failure. So stopping it
  without first disabling the policy means it is back within a second and has
  taken the camera again -- which looks like "the camera is busy for no reason".

  DO NOT exec. An earlier runner ended with `exec python3 ...`. exec REPLACES the
  shell, which destroys the EXIT trap, so narsil-vision was left stopped with
  restart=no and the camera never came back until someone noticed. In Python the
  equivalent mistake is os.execv or an os._exit that skips the finally block.

Use the context manager and the restore happens on exceptions and signals too.
"""
import contextlib
import subprocess
import time

CONTAINER = 'narsil-vision'
SETTLE_S = 4.0


def _docker(*args):
    return subprocess.run(['docker'] + list(args), stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def is_running(container=CONTAINER):
    r = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', container],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    return r.stdout.decode().strip() == 'true'


def stop_vision(container=CONTAINER, settle=SETTLE_S):
    _docker('update', '--restart=no', container)
    _docker('stop', container)
    time.sleep(settle)


def start_vision(container=CONTAINER):
    _docker('update', '--restart=on-failure', container)
    _docker('start', container)


@contextlib.contextmanager
def vision_stopped(container=CONTAINER, settle=SETTLE_S, verbose=True):
    """Hold the camera for the duration of the block, then hand it back."""
    was = is_running(container)
    if was:
        if verbose:
            print('[%s stopped, camera released]' % container, flush=True)
        stop_vision(container, settle)
    try:
        yield
    finally:
        if was:
            start_vision(container)
            if verbose:
                print('[%s restored]' % container, flush=True)
