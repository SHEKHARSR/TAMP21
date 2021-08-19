#!/usr/bin/env python

import rospy
import numpy as np

from svea.svea_managers.tamp_svea import TAMP_svea_manager
from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.controllers.tamp_control import TAMPController
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA_copy import SimSVEA
from svea_msgs.msg import lli_ctrl
from svea_msgs.msg import lli_emergency
## SIMULATION PARAMS ##########################################################
vehicle_name = ""
#target_velocity = 3.1  # [m/s]
dt = 0.01  # frequency of the model updates
## INIT #######################################################################
# [x, y, yaw, v], units: [m, m, rad, m/s]
default_init_pt = [0.0, 0.0, 0.0, 0.0]
###############################################################################

class Tamp_svea:
    def __init__(self):
        rospy.init_node('tamp_svea')
        """ Initialization handles use with just python or in a launch file """


        # grab parameters from launch-file
        start_pt_param = rospy.search_param('start_pt')
        is_sim_param = rospy.search_param('is_sim')
        use_rviz_param = rospy.search_param('use_rviz')
        use_matplotlib_param = rospy.search_param('use_matplotlib')


        start_pt = rospy.get_param(start_pt_param, default_init_pt)
        if isinstance(start_pt, str):
            start_pt = start_pt.split(',')
            start_pt = [float(curr) for curr in start_pt]
            start_pt = VehicleState(*start_pt)

        self.is_sim = rospy.get_param(is_sim_param, True)
        self.use_rviz = rospy.get_param(use_rviz_param, True)
        self.use_matplotlib = rospy.get_param(use_matplotlib_param, True)

        
        if not self.use_rviz:
            self.DataHandler = RVIZPathHandler
        elif self.use_matplotlib:
            self.DataHandler = TrajDataHandler
        else:
            self.DataHandler = RVIZPathHandler
            #self.DataHandler = BasicDataHandler     
        if self.is_sim:
        # start the simulation
            model_for_sim = SimpleBicycleModel(start_pt)
            self.simulator = SimSVEA(model_for_sim, vehicle_name,dt=dt, run_lidar=True, start_paused=True).start()
    def run_main(self):        # start TAMP SVEA manager
        self.svea = TAMP_svea_manager(vehicle_name,
                           LocalizationInterface,
                           TAMPController)
        self.svea.start(wait=True)

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

    # simualtion loop
        while not self.svea.is_finished and not rospy.is_shutdown():
            state = self.svea.wait_for_state()

            # get control input via control interface node
            steering, velocity = self.svea.get_control()
            self.svea.send_control(steering, velocity,transmission = 1)
            #print(self.svea.get_control())
            #print(self.simulator._build_param_printout())
            # visualize data
            #self.use_matplotlib or
            if self.use_rviz:
                self.svea.visualize_data()
            else:
                rospy.loginfo_throttle(1, state)

        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished!")

        rospy.spin()
if __name__ == '__main__':
    try:
        node_name = Tamp_svea()
        node_name.run_main()
    except rospy.ROSInterruptException:
        pass
