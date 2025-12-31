import os
import signal
import subprocess
import time
from typing import Optional

def start_process(cmd: str, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) -> subprocess.Popen:
    """
    Start a process in a new process group (setsid).
    Default stdout/stderr are purely local devnull (not preventing pipe deadlock, just ignoring).
    """
    return subprocess.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setsid,
        stdout=stdout,
        stderr=stderr,
        text=True,
    )

def stop_process_group(proc: subprocess.Popen, timeout_sec: float = 5.0) -> None:
    """
    Safely stop a process group led by 'proc'.
    Tries SIGINT -> wait -> SIGKILL.
    """
    if proc.poll() is not None:
        return

    try:
        pgid = os.getpgid(proc.pid)
    except Exception:
        # Process dead or permission denied
        proc.terminate()
        return

    # 1. SIGINT
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    except Exception:
        pass

    # 2. SIGKILL
    try:
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass

    # 3. Final Reap
    try:
        proc.wait(timeout=1.0)
    except Exception:
        pass
