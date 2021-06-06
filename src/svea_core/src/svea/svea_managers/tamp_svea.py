#!/usr/bin/env python
""" Module for TAMP based SVEA Manager"""

import numpy as np
import rospy

from math import cos, sin, sqrt

from svea_archetypes import SVEAManager
#from path_following_sveas import SVEAPurePursuit
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler

class TAMP_svea_manager(SVEAManager):
    """ Container for TAMP_SVEA"""

    def __init__(self, vehicle_name, localizer, controller, data_handler = RVIZPathHandler):
        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler=data_handler)

        
        #self.update_traj(traj_x, traj_y)

        #goto
        self.goto_thresh = 0.05  # m
        self.goto_vel = 0.6  # m/s
    def get_control1(self):
        """To get published inputes from ctrl node """
        steering, velocity = self.svea.controller.get_control()
        #steering = #self.get_steering
        #velocity = #self.get_velocity 
        return steering,velocity
    
    def get_control(self, state=None):
        """
        :param state: State used to compute control; if no state is
                      given as an argument, self.state is automatically
                      used instead, defaults to None
        :type state: VehicleState, or None

        :return: Computed steering and velocity inputs from TAMP from selected control
                 algorithm
        :rtype: float, float
        """
        if state is None:
            steering, velocity = self.controller.get_control()
            self.data_handler.update_target(self.controller.target)
        else:
            steering, velocity = self.controller.get_control()
            self.data_handler.update_target(self.controller.target)
        return steering, velocity
    @property
    def is_finished(self):
        """Check if  controller is finished or not"""
        return self.controller.is_finished