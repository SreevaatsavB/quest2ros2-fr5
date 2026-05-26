"""
gripper.py — DH AG-160-95 control from the Quest trigger, via the Fairino SDK.

MoveGripper returns error 73 while ServoMoveStart is active. So a state change
can't be sent mid-servo: the background poll thread raises a flag, the control
loop pauses ServoJ, the gripper command is sent, then servo resumes (~400 ms
freeze per open/close). Same handshake that worked in the prior project.
"""

import threading
import time


class GripperController:
    POLL_HZ = 5

    def __init__(self, driver, *, index, gtype, open_pct, close_pct,
                 vel_pct, force_pct, maxtime_ms, open_thr, close_thr):
        self._driver = driver
        self._index = index
        self._gtype = gtype
        self._open_pct = open_pct
        self._close_pct = close_pct
        self._vel_pct = vel_pct
        self._force_pct = force_pct
        self._maxtime_ms = maxtime_ms
        self._open_thr = open_thr
        self._close_thr = close_thr

        self._thread = None
        self._stop_evt = threading.Event()
        self._state = None              # "open" | "closed" | None

        self._norm = 0.0
        self._norm_valid = False
        self._norm_lock = threading.Lock()

        self._cmd_ready = threading.Event()
        self._servo_paused = threading.Event()
        self._cmd_done = threading.Event()

    # ── called from control loop ──────────────────────────────────────────────

    def update_trigger(self, norm: float):
        with self._norm_lock:
            self._norm = max(0.0, min(1.0, float(norm)))
            self._norm_valid = True

    def wants_pause(self) -> bool:
        return self._cmd_ready.is_set()

    def pause_and_send(self):
        """Pause servo, send the pending gripper command, resume servo."""
        self._driver.stop_servo_mode()
        time.sleep(0.2)
        self._driver.reset_errors()
        time.sleep(0.1)

        self._servo_paused.set()
        self._cmd_done.wait(timeout=2.0)
        self._cmd_done.clear()
        self._servo_paused.clear()
        self._cmd_ready.clear()

        self._driver.enable()
        self._driver.start_servo_mode()

    # ── lifecycle ──────────────────────────────────────────────────────────────

    def start(self) -> bool:
        err = self._driver.activate_gripper(self._index)
        if err != 0:
            print(f"[GRIPPER] ActGripper failed (err={err}) — check FR5 gripper config")
            return False
        time.sleep(0.5)
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[GRIPPER] DH AG-160-95 active (index={self._index})")
        return True

    def stop(self):
        self._stop_evt.set()
        self._servo_paused.set()   # unblock a pending handshake
        if self._thread:
            self._thread.join(timeout=2)

    # ── background thread ────────────────────────────────────────────────────────

    def _loop(self):
        interval = 1.0 / self.POLL_HZ
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            with self._norm_lock:
                norm, valid = self._norm, self._norm_valid
            if not valid:
                self._stop_evt.wait(timeout=interval)
                continue

            desired = None
            if norm >= self._close_thr:
                desired = "closed"
            elif norm <= self._open_thr:
                desired = "open"

            if desired and desired != self._state:
                pct = self._open_pct if desired == "open" else self._close_pct
                self._cmd_ready.set()
                self._servo_paused.wait(timeout=3.0)
                if self._stop_evt.is_set():
                    break
                try:
                    err = self._driver.send_gripper(
                        self._index, pct, self._vel_pct, self._force_pct,
                        self._maxtime_ms, 1, self._gtype,
                    )
                    if err == 0:
                        self._state = desired
                        print(f"[GRIPPER] {desired.upper()} (trig={norm:.2f})")
                    else:
                        print(f"[GRIPPER] MoveGripper error {err}")
                except Exception as exc:
                    print(f"[GRIPPER] exception: {exc}")
                finally:
                    self._cmd_done.set()

            remaining = interval - (time.monotonic() - t0)
            if remaining > 0:
                self._stop_evt.wait(timeout=remaining)
