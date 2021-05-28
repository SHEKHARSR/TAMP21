#!/usr/bin/env python
""" Module for TAMP based SVEA Manager"""

import numpy as np
import rospy

from math import cos, sin, sqrt

from svea_archetypes import SVEAManager
from svea.data import TrajDataHandler

class TAMP_svea_manager(SVEAManager):
    """ Container for TAMP_SVEA"""

    def __init__(self, vehicle_name, localizer, controller,traj_x, traj_y, data_handler=TrajDataHandler):
        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler=data_handler)

        self.update_traj(traj_x, traj_y)

        #goto
        self.goto_thresh = 0.05  # m
        self.goto_vel = 0.6  # m/s
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




        

    





