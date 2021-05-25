#!/usr/bin/env python

import rospy
import numpy as np

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA_copy import SimSVEA
from svea.track import Track


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 0.3 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [0.0, 0.29389262614623657, 0.5877852522924731, 0.8816778784387097, 1.1755705045849465, 1.469463130731183, 1.7633557568774196, 2.0572483830236563, 2.351141009169893, 2.6450336353161297, 2.9389262614623664, 3.2328188876086026, 3.5267115137548393, 3.820604139901076, 4.114496766047313, 4.408389392193549, 4.702282018339786, 4.996174644486023, 5.290067270632259, 5.583959896778496, 5.877852522924733, 6.116822111087557, 6.241366833152671, 6.244075019107561, 6.121846312882621, 5.883371732256978, 5.545101255797817, 5.126833976461284, 4.653627565749882, 4.156611572308676, 3.6655578498803303, 3.212033035546465, 2.8239212877110895, 2.5130304068120726, 2.2191377806658337, 1.9252451545195959, 1.6313525283733583, 1.33745990222712, 1.0435672760808823, 0.7496746499346445, 0.4557820237884067, 0.16188939764216898, -0.132003228504069, -0.42589585465030677, -0.7197884807965448, -1.0136811069427825, -1.3075737330890205, -1.6014663592352583, -1.8953589853814963, -2.189251611527734, -2.4831442376739723, -2.7770368638202094, -3.0709294899664457, -3.364822116112683, -3.658714742258919, -3.952607368405155, -4.246499994551391, -4.540392620697628, -4.834285246843865, -5.128177872990102, -5.422070499136338, -5.7049160866363104, -5.896641793192495, -5.966780068955817, -5.9140582614922, -5.741594586002847, -5.455968269986297, -5.079112298774014, -4.6338241971129985, -4.146493688269942, -3.6484057049203145, -3.1701672766567346, -2.7408889267986294, -2.3897993454166873, -2.0936327319441066, -1.7997401057978695, -1.505847479651632, -1.2119548535053943, -0.9180622273591565, -0.6241696012129192, -0.3302769750666831, -0.03638434892044767]#[-7.4 , -4.5 , 10.3, 5.6, -7.2, -13.5]
ys = [0.0, 0.4045084971874737, 0.8090169943749475, 1.2135254915624212, 1.6180339887498951, 2.022542485937369, 2.427050983124843, 2.8315594803123165, 3.2360679774997902, 3.6405764746872644, 4.045084971874738, 4.449593469062211, 4.854101966249686, 5.25861046343716, 5.663118960624633, 6.067627457812107, 6.4721359549995805, 6.876644452187055, 7.281152949374529, 7.685661446562002, 8.090169943749476, 8.527863313091494, 9.010413505774052, 9.50890268676198, 9.99234650336419, 10.430034437444258, 10.796044693832492, 11.067557947730315, 11.224612397169642, 11.261107978719318, 11.174863675478097, 10.967548079429614, 10.65487048896864, 10.264096084346773, 9.859587587159298, 9.455079089971822, 9.050570592784348, 8.646062095596871, 8.241553598409395, 7.837045101221921, 7.432536604034445, 7.02802810684697, 6.623519609659494, 6.219011112472018, 5.814502615284543, 5.409994118097068, 5.0054856209095915, 4.600977123722116, 4.196468626534641, 3.791960129347166, 3.3874516321596895, 2.982943134972215, 2.5784346377847416, 2.1739261405972674, 1.7694176434097946, 1.3649091462223208, 0.960400649034848, 0.5558921518473733, 0.1513836546599001, -0.25312484252757445, -0.657633339715048, -1.069517574552667, -1.5298435079624269, -2.023396527225986, -2.518959924697292, -2.9868447981842507, -3.395618069657157, -3.721731283459059, -3.9457913429963636, -4.051461430240757, -4.032462207943961, -3.892206669572606, -3.638455106795209, -3.2844129471694643, -2.88163824035454, -2.477129743167065, -2.07262124597959, -1.6681127487921144, -1.2636042516046393, -0.8590957544171642, -0.4545872572296912, -0.050078760042219006]#[-15.3,  -11 , 11.5, 14.5, -4.1, -11.6]
traj_x = []
traj_y = []
for i in range(0,len(xs)-1):
    traj_x += np.linspace(xs[i], xs[i+1]).tolist()
    traj_y += np.linspace(ys[i], ys[i+1]).tolist()
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################


def param_init():
    """Initialization handles use with just python or in a launch file
    """
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

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)

    return start_pt, is_sim, use_rviz, use_matplotlib


def main():
    rospy.init_node('floor2_example')
    start_pt, is_sim, use_rviz, use_matplotlib = param_init()

    # select data handler based on the ros params
    if use_rviz:
        DataHandler = RVIZPathHandler
    else:
        DataHandler = TrajDataHandler

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(model_for_sim,vehicle_name,
                            dt=dt, run_lidar=False, start_paused=True).start()

    # start pure pursuit SVEA manager
    svea = SVEAPurePursuit(vehicle_name,
                           LocalizationInterface,
                           PurePursuitController,
                           traj_x, traj_y,
                           data_handler = DataHandler)
    svea.start(wait=True)

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)
        
        # compute control input via control interface
        #TODO write subscriber to the saarti node
        #steering, velocity = saarti.saarti.compute_control()
        #svea.send_control(steering, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
