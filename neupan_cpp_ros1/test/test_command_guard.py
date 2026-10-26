#!/usr/bin/env python3
"""Execution guard tests without ROS or chassis hardware."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from command_guard import CommandGuard, fw_fields


class GuardTests(unittest.TestCase):
    def guard(self):
        return CommandGuard(dict(max_speed=[.2,.4], max_acce=[.3,.6]))

    def test_enable_needs_fresh_command_and_repeat_false_is_harmless(self):
        g = self.guard()
        g.receive(.2,.4,0.)
        g.enable(True)
        self.assertEqual(g.tick(.1,.02),(0.,0.))
        g.set_emergency(False)
        g.receive(.2,.4,.2)
        self.assertEqual(g.tick(.2,.02),(.006,.012))

    def test_timeout_ramps_and_does_not_resume_old_command(self):
        g = self.guard(); g.enable(True)
        previous = (0.,0.)
        for i in range(100):
            now = i*.02
            if i < 40: g.receive(.2,-.4,now)
            out = g.tick(now,.02)
            for j, bound in enumerate([.3,.6]):
                self.assertLessEqual(abs(out[j]-previous[j])/.02,bound+1e-10)
            previous = out
        self.assertEqual(out,(0.,0.))

    def test_invalid_lateral_and_emergency_stop_disable(self):
        for v,w,y in [(float('nan'),0.,0.),(.1,.2,.01)]:
            g=self.guard();g.enable(True)
            self.assertFalse(g.receive(v,w,0.,y))
            self.assertEqual(g.tick(.02,.02),(0.,0.))
            self.assertFalse(g.enable(True))
            g.set_emergency(False)
            self.assertFalse(g.enabled)
        g=self.guard();g.enable(True);g.receive(.2,.4,0.)
        g.tick(.02,.02);g.set_emergency(True)
        self.assertEqual(g.applied,(0.,0.))

    def test_fw_units_and_required_gear(self):
        self.assertEqual(fw_fields((.2,math.pi/2),2),(2,.2,90.,0.))
        for gear in [None, -1, 16, True, '2']:
            with self.assertRaises(ValueError): fw_fields((0.,0.),gear)


if __name__ == '__main__': unittest.main()
