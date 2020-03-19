#!/usr/bin/env python
# ======================================================
# Copyright (C) 2020 repa1030
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================
'''
Core calculation of pure pursuit algorithm
The algorithm calculates a circle through the
current position of the car and the next waypoint.
This returns a curvature which can be converted to the
single track steering model.
'''

import math
import numpy as np

class PurePursuit:

    # initialization of instance of pure pursuit
    def __init__(self, cmd_velo, vehicle_mass, wheel_rad, motor_torque, 
                    brake_torque, wheel_base, offset):
        self.next_wp = np.zeros(3) # [x1, y1, z1], z1 is not used
        self.current_pos = np.zeros(3) # [x0, y0, z0], z0 is not used
        self.max_radius = 9e10 # max radius of steering
        self.wheel_base = wheel_base # wheel base [m], used for Ackermann
        self.offset = offset # offset from vehicle to clipping plane
        self.wp_velocity = cmd_velo * 1000.0 / (60.0 * 60.0)
        self.max_accel = motor_torque / wheel_rad / vehicle_mass
        self.max_decel = brake_torque / wheel_rad / vehicle_mass
        self.gas = 0.0      # init
        self.brake = 0.0    # init
        self.steer = 0.0    # init

    # this converts the curvature to a steering angle
    def convertToAckermann(self, kappa):
        return -math.atan(self.wheel_base * kappa)
 
    # this can be used to set the current pose
    def updatePos(self, pose):
        self.current_pose = pose
    
    # this can be used to set the new waypoint
    def updateWp(self, wp):
        self.next_wp[0] = wp[0] + self.offset
        self.next_wp[1] = wp[1]
        self.next_wp[2] = wp[2]

    # this calculates the curvature (pure pursuit algorithm)
    def calcCurvature(self, dist, y):
        if y == 0 or dist == 0:
            curvature = 1 / self.max_radius
        else:
            curvature = 2 * y / dist / dist            
            if abs(curvature) < 1 / self.max_radius:
                curvature = 1 / self.max_radius
        return curvature

    # this calculates the current distance from pose to waypoint
    def calcDistance(self):
        x = self.next_wp[0] - self.current_pos[0]
        y = self.next_wp[1] - self.current_pos[1]
        dst = math.sqrt(x*x + y*y)    
        return dst, y

    # this is the pipeline for calculating the steering angle
    # make sure, that current pose and waypoint is set before
    # calling this function
    def calcSteeringAngle(self):
        (dst, y) = self.calcDistance()
        kappa = self.calcCurvature(dst, y)
        self.steer = self.convertToAckermann(kappa)
        return dst

    # this calculates the gas and brake pedal positions
    # based on current velocity and dyn_thresh parameter
    # a threshold is calculated and compared to
    # v^2 - v0^2 = 2ax => a = (v^2 - v0^2) / (2x)
    # based on the result, the vehicle accelerates or brakes
    def calcPedalPositions(self, current_velo, dst):
        a = (self.wp_velocity * self.wp_velocity - current_velo * current_velo) / (2.0 * dst)
        if (a < 0.0):
            # brake
            self.gas = 0.0
            self.brake = abs(a / self.max_decel)
            if self.brake > 1.0:
                self.brake = 1.0
        else:
            # accelerate
            self.gas = abs(a / self.max_accel)
            self.brake = 0.0
            if self.gas > 1.0:
                self.gas = 1.0

    # use this function to calculate control commands
    def calcControlCommands(self, current_velo, waypoint):
        self.updateWp(waypoint)        
        dst = self.calcSteeringAngle()
        self.calcPedalPositions(current_velo, dst)
        return (round(self.steer, 4), round(self.brake, 4), round(self.gas, 4))
