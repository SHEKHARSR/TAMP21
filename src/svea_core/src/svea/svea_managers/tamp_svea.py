#!/usr/bin/env python
""" Module for TAMP based SVEA Manager"""

import numpy as np
import rospy

from math import cos, sin, sqrt

from svea_archetypes import SVEAManager
from path_following_sveas import SVEAPurePursuit
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler

class TAMP_svea_manager(SVEAManager):
    """ Container for TAMP_SVEA"""
    steering = 0
    velocity = 0

    def __init__(self, vehicle_name, localizer, controller,traj_x, traj_y, data_handler = RVIZPathHandler):
        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler=data_handler)

        
        self.update_traj(traj_x, traj_y)

        #goto
        self.goto_thresh = 0.05  # m
        self.goto_vel = 0.6  # m/s
    def get_control(self):
        """To get published inputes from ctrl node """
        #steering = #self.get_steering
        #velocity = #self.get_velocity 
        return self.steering,self.velocity
    

    def goto_pt(self, pt):
        """Compute control to go to single point, taking advantage of
        the pure-pursuit controller

        :param pt: Point to go to
        :type pt: tuple
        :return: Computed steering and velocity inputs from pure-pursuit
                 algorithm
        :rtype: float, float
        """
        curr_xy = [self.state.x, self.state.y]
        target_xy = (pt[0], pt[1])
        dist = math.sqrt((curr_xy[0] - target_xy[0])**2
                         + (curr_xy[1] - target_xy[1])**2)

        if dist > self.goto_thresh:
            steering, velocity = \
                self.controller.compute_control(self.state, target_xy)
            self.data_handler.update_target(self.controller.target)
            return steering, velocity
        else:
            steering = 0.0
            velocity = 0.0
            return steering, velocity

    def update_traj(self, traj_x, traj_y):
        """Update trajectory

        :param traj_x: X coordinates of trajectory, defaults to []
        :type traj_x: list
        :param traj_y: Y coordinates of trajectory, defaults to []
        :type traj_y: list
        """

        assert len(traj_x) == len(traj_y)
        self.controller.traj_x = traj_x
        self.controller.traj_y = traj_y
        self.data_handler.update_traj(traj_x, traj_y)

    @property
    def is_finished(self):
        """Check if pure-pursuit controller is finished or not"""
        return self.controller.is_finished
