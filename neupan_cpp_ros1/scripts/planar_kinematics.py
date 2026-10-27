#!/usr/bin/env python3
"""Kinematic virtual plant. Omni polar bounds match the C++/Python optimizer.
The ROS interface always uses body-frame velocity; omni has zero yaw rate.
"""
import math
from command_guard import limit_command


def rotate_vector(vector, yaw):
    x, y = vector
    return (math.cos(yaw)*x-math.sin(yaw)*y,
            math.sin(yaw)*x+math.cos(yaw)*y)


def control_difference(current, previous, robot):
    angle = current[1]-previous[1]
    if robot.get('periodic_direction',False):
        angle = math.atan2(math.sin(angle),math.cos(angle))
    return current[0]-previous[0], angle


class PlanarPlant:
    def __init__(self, robot):
        self.model = robot['kinematics']
        if self.model not in ('diff', 'omni'):
            raise ValueError('kinematics must be diff or omni')
        self.robot = robot
        self.control = (0., 0.)

    def reset(self):
        self.control = (0., 0.)

    def apply(self, action, yaw, dt, emergency=False):
        if emergency:
            self.control = (0., self.control[1] if self.model == 'omni' else 0.)
            return (0., 0.)
        requested = action
        if self.model == 'omni':
            vx, vy = rotate_vector(action, yaw)
            speed = math.hypot(vx, vy)
            if speed < 1e-10:
                requested = (0., self.control[1])
            else:
                angle = math.atan2(vy, vx)
                candidates = []
                for sign in (1, -1):
                    phi = angle+(math.pi if sign < 0 else 0.)
                    center = round((self.control[1]-phi)/(2*math.pi))
                    for offset in (-1, 0, 1):
                        direction = phi+(center+offset)*2*math.pi
                        if abs(direction) <= self.robot['max_speed'][1]+1e-6:
                            candidates.append((sign*speed, direction))
                if not candidates:
                    raise ValueError('omni direction cannot be represented within angle bound')
                requested = min(candidates, key=lambda u: sum(
                    ((u[i]-self.control[i])/self.robot['max_acce'][i])**2 for i in (0, 1)))
        self.control = limit_command(requested, self.control, self.robot['max_speed'],
                                     self.robot['max_acce'], dt)
        if self.model == 'omni' and self.robot.get('periodic_direction',False):
            self.control = (self.control[0],math.atan2(math.sin(self.control[1]),math.cos(self.control[1])))
        if self.model == 'diff':
            return self.control
        speed, direction = self.control
        return rotate_vector((speed*math.cos(direction), speed*math.sin(direction)), -yaw)

    def integrate(self, state, action, dt):
        x, y, yaw = state
        if self.model == 'omni':
            vx, vy = rotate_vector(action, yaw)
            return [x+vx*dt, y+vy*dt, yaw]
        v, w = action
        # Match the existing exact unicycle integration.
        if abs(w) < 1e-9:
            return [x+v*math.cos(yaw)*dt, y+v*math.sin(yaw)*dt, yaw+w*dt]
        return [x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
                y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)), yaw+w*dt]
