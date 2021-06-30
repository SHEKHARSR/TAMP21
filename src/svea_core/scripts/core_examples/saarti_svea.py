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

#TODO: create a trajectory that goes around the track
#xs = [0.0, 0.30021011266294206, 0.6004202253258841, 0.9006303379888262, 1.2008404506517683, 1.5010505633147104, 1.8012606759776524, 2.1014707886405946, 2.4016809013035365, 2.701891013966479, 3.002101126629421, 3.302311239292363, 3.6025213519553048, 3.902731464618247, 4.202941577281189, 4.503151689944131, 4.753128077876667, 4.904806653851315, 4.951581485934778, 4.883338771994047, 4.707749300595965, 4.436529539626753, 4.078909938086989, 3.6516163384337186, 3.1795938238466173, 2.683634034705587, 2.188799200196997, 1.719644075095498, 1.2968409437926576, 0.945961674706408, 0.6425100884611774, 0.342299975798236, 0.04208986313529395, -0.25812024952764845, -0.5583303621905905, -0.8585404748535325, -1.1587505875164745, -1.4589607001794151, -1.759170812842357, -2.0593809255052995, -2.3595910381682415, -2.659801150831184, -2.960011263494126, -3.2602213761570686, -3.5604314888200115, -3.8606416014829525, -4.160851714145895, -4.461061826808837, -4.761271939471779, -5.061482052134722, -5.361692164797665, -5.6223141876935445, -5.789754213417274, -5.848498577698452, -5.793076253164695, -5.633136568132312, -5.37492343244126, -5.023806332823568, -4.6055829039973535, -4.138727253487943, -3.6437659411254675, -3.1476730208964954, -2.6730220819973627, -2.2439631402245306, -1.8840425718860228, -1.5773495120585042, -1.2771393993955622, -0.9769292867326198, -0.6767191740696777, -0.37650906140673557, -0.0762989487437935]# [-7.4 , -4.5 , 10.3, 5.6, -7.2, -13.5]
#ys = [0.0, 0.39984232924354524, 0.7996846584870905, 1.1995269877306358, 1.5993693169741812, 1.9992116462177263, 2.3990539754612716, 2.798896304704817, 3.1987386339483623, 3.5985809631919077, 3.9984232924354526, 4.3982656216789975, 4.798107950922543, 5.197950280166088, 5.597792609409633, 5.997634938653179, 6.429463623719977, 6.904069570345925, 7.400489655734264, 7.894734389524635, 8.361141607727319, 8.77926523340674, 9.12717545325528, 9.384095395460633, 9.54365469630982, 9.598273576841503, 9.534581775254896, 9.366817197039666, 9.102444491844139, 8.747732398130804, 8.35049395270503, 7.950651623461484, 7.55080929421794, 7.150966964974394, 6.751124635730849, 6.351282306487303, 5.951439977243759, 5.551597648000215, 5.1517553187566705, 4.751912989513125, 4.35207066026958, 3.952228331026034, 3.552386001782488, 3.1525436725389424, 2.7527013432953966, 2.3528590140518517, 1.9530166848083061, 1.5531743555647604, 1.1533320263212157, 0.7534896970776697, 0.353647367834124, -0.07171503022338904, -0.541154112771643, -1.036614663523396, -1.5320785408921762, -2.0039815728924912, -2.4309251517263637, -2.7853125281427062, -3.0561195788936932, -3.2314403294919654, -3.294310177522639, -3.247157330041732, -3.095042538192195, -2.840406285889397, -2.4954668664835666, -2.1008323048755657, -1.7009899756320206, -1.3011476463884752, -0.9013053171449299, -0.5014629879013845, -0.1016206586578392] # [-15.3,  -11 , 11.5, 14.5, -4.1, -11.6]
#xs = []
#ys = []


#traj_x = []
#traj_y = []
#for i in range(0, len(xs)-1):
#    traj_x += np.linspace(xs[i], xs[i+1]).tolist()
#    traj_y += np.linspace(ys[i], ys[i+1]).tolist()
###############################################################################

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
                           #traj_x, traj_y)
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
            #print(self.svea.actuation._build_param_printout())
            print(self.simulator._build_param_printout())
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