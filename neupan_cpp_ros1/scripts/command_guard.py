#!/usr/bin/env python3
"""Transport independent command guard. SI units; a command is not odometry."""
import math


def limit_command(target, previous, max_speed, max_acce, dt):
    if (len(target) != 2 or len(previous) != 2 or len(max_speed) != 2 or
            len(max_acce) != 2 or not all(math.isfinite(x) for x in
            [*target, *previous, *max_speed, *max_acce, dt]) or dt <= 0 or
            min(*max_speed, *max_acce) <= 0):
        raise ValueError('invalid command, limits or elapsed time')
    return tuple(max(previous[i]-max_acce[i]*dt,
                     min(previous[i]+max_acce[i]*dt,
                         max(-max_speed[i], min(max_speed[i], target[i])))) for i in range(2))


class CommandGuard:
    """Fresh commands must arrive after enabling. Safety faults latch a stop."""
    def __init__(self, robot, timeout=.3):
        self.speed, self.acce = robot['max_speed'], robot['max_acce']
        limit_command((0., 0.), (0., 0.), self.speed, self.acce, .02)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive')
        self.timeout, self.enabled, self.emergency = timeout, False, False
        self.target, self.applied, self.receipt = (0., 0.), (0., 0.), None

    def enable(self, enabled):
        if enabled and self.emergency:
            return False
        self.enabled = enabled
        self.receipt, self.target = None, (0., 0.)
        if not enabled:
            self.applied = (0., 0.)  # explicit operator stop
        return True

    def receive(self, v, w, now, lateral=0.):
        if not all(math.isfinite(x) for x in (v, w, lateral, now)) or abs(lateral) > 1e-6:
            self.set_emergency(True)
            return False
        if self.enabled:
            self.target, self.receipt = (v, w), now
        return True

    def set_emergency(self, active):
        if active or self.emergency:
            self.emergency = bool(active)
            self.enable(False)  # clearing a fault never resumes motion

    def tick(self, now, dt):
        if self.emergency or not self.enabled:
            self.applied = (0., 0.)
        else:
            fresh = self.receipt is not None and 0 <= now-self.receipt <= self.timeout
            self.applied = limit_command(self.target if fresh else (0., 0.),
                                         self.applied, self.speed, self.acce, dt)
        return self.applied


def fw_fields(command, gear):
    if isinstance(gear, bool) or not isinstance(gear, int) or not 0 <= gear <= 15:
        raise ValueError('ctrl_gear must be an explicit firmware gear in [0,15]')
    if not all(math.isfinite(x) for x in command):
        raise ValueError('non-finite FW-mini command')
    # The supplied driver multiplies this angular field by 100 directly.
    return gear, command[0], math.degrees(command[1]), 0.
