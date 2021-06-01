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
target_velocity = 3.1  # [m/s]
dt = 0.01  # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [0.0, 0.29389262614623657, 0.5877852522924731, 0.8816778784387097, 1.1755705045849463, 1.469463130731183, 1.7633557568774192, 2.0572483830236554, 2.351141009169892, 2.6450336353161292, 2.9389262614623655, 3.2328188876086017, 3.526711513754838, 3.8206041399010746, 4.114496766047311, 4.4083893921935475, 4.7476250945618625, 5.156650525318334, 5.61737168849959, 6.1098334624591555, 6.607252608946061, 7.086641796417417, 7.524326538271804, 7.8955303734057996, 8.18682969230972, 8.38486621449918, 8.46949143111796, 8.4470937446985, 8.318729330006802, 8.0828706429705, 7.792419914735004, 7.498527288588768, 7.2046346624425315, 6.910742036296296, 6.616849410150058, 6.322956784003822, 6.029064157857586, 5.735171531711348,
      5.441278905565111, 5.147386279418875, 4.853493653272638, 4.559601027126401, 4.265708400980165, 3.971815774833927, 3.6779231486876904, 3.3840305225414538, 3.0901378963952175, 2.7962452702489804, 2.502352644102744, 2.208460017956507, 1.9145673918102701, 1.584642367364037, 1.1851898910117866, 0.7289053694739291, 0.23904805806891608, -0.2590222805836139, -0.7430170453438798, -1.1849806104215355, -1.5667906509192933, -1.8716353870026452, -2.077264533856265, -2.1780674952157306, -2.1722437561641574, -2.055024835681427, -1.83116530905679, -1.544156478732035, -1.2502638525857983, -0.9563712264395616, -0.6624786002933251, -0.3685859741470884, -0.07469334800085181]  # [-7.4 , -4.5 , 10.3, 5.6, -7.2, -13.5]
ys = [0.0, 0.4045084971874737, 0.8090169943749475, 1.2135254915624214, 1.618033988749895, 2.0225424859373686, 2.4270509831248424, 2.831559480312316, 3.2360679774997894, 3.640576474687263, 4.045084971874736, 4.44959346906221, 4.854101966249684, 5.258610463437158, 5.663118960624632, 6.067627457812105, 6.43352819026422, 6.718054800578426, 6.9087293817116, 6.988831354684242, 6.958146694580836, 6.821861948453279, 6.582351265542516, 6.249491851090614, 5.8452528975362545, 5.387278148849763, 4.895590512494407, 4.3978457699246905, 3.9159972541723396, 3.4763296403428567, 3.069488612192425, 2.6649801150049504, 2.260471617817477, 1.8559631206300036, 1.4514546234425298, 1.0469461262550561, 0.6424376290675816, 0.23792913188010778, -
      0.16657936530736622, -0.5710878624948401, -0.9755963596823141, -1.380104856869788, -1.7846133540572622, -2.189121851244736, -2.5936303484322107, -2.9981388456196836, -3.402647342807158, -3.807155839994633, -4.211664337182106, -4.61617283436958, -5.020681331557054, -5.394855738409969, -5.692927765903606, -5.894760600425147, -5.9874746894142685, -5.973363976678752, -5.852104294032226, -5.620713202261067, -5.300606363641984, -4.905940364491846, -4.451346070102621, -3.9633190836479484, -3.464923148912606, -2.9799582829863174, -2.534522618818596, -2.1253490597052034, -1.7208405625177297, -1.316332065330256, -0.911823568142782, -0.507315070955308, -0.10280657376783409]  # [-15.3,  -11 , 11.5, 14.5, -4.1, -11.6]
traj_x = []
traj_y = []
for i in range(0, len(xs)-1):
    traj_x += np.linspace(xs[i], xs[i+1]).tolist()
    traj_y += np.linspace(ys[i], ys[i+1]).tolist()
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

        #This subscribes to ctrl node
        #TODO
        #control_update_sub = rospy.Subscriber('/lli/ctrl_request', lli_ctrl, self.callback_ctrl_interface)
        #control_update_sub = rospy.Subscriber('/lli/ctrl_actuated', lli_ctrl, self.callback_ctrl_interface_actuated)


        
        if self.is_sim:
        # start the simulation
            model_for_sim = SimpleBicycleModel(start_pt)
            self.simulator = SimSVEA(model_for_sim, vehicle_name,dt=dt, run_lidar=True, start_paused=True).start()


    def callback_ctrl_interface(self,lli_ctrl):
        self.get_steering = lli_ctrl.steering
        self.get_velocity = lli_ctrl.velocity
    
    def callback_ctrl_interface_actuated(self,lli_ctrl):
        self.get_steering = lli_ctrl.steering
        self.get_velocity = lli_ctrl.velocity



    def run_main(self):        # start TAMP SVEA manager
        self.svea = TAMP_svea_manager(vehicle_name,
                           LocalizationInterface,
                           TAMPController,
                           traj_x, traj_y)
        self.svea.start(wait=True)

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

    # simualtion loop
        self.svea.controller.target_velocity = target_velocity
        while not self.svea.is_finished and not rospy.is_shutdown():
            state = self.svea.wait_for_state()

            # get control input via control interface node
            #steering = self.get_steering
            #velocity = self.get_velocity
            #self.svea.get_control()
            steering, velocity = self.svea.get_control()
            self.svea.send_control(steering, velocity)

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