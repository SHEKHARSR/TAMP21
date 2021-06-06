import rospy
import math
from svea_msgs.msg import lli_ctrl
from svea_msgs.msg import lli_emergency
from svea_msgs.msg import tamp_control as tc

class TAMPController(object):

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.last_time = 0.0
        self.recieved_steering = 0.0
        self.recieved_velocity = 0.0
        #self.control_update_sub = rospy.Subscriber('/lli/ctrl_request', lli_ctrl, self.callback_ctrl_interface)
        self.control_update_sub = rospy.Subscriber('/Control_signal', tc, self.callback_ctrl_from_tamp)
        #self.tamp_control = tamp_control()
        self.received_tamp_control = False 
        

    """def callback_ctrl_interface(self,lli_ctrl):
        self.get_steering = lli_ctrl.steering
        self.get_velocity = lli_ctrl.velocity"""
    
    def callback_ctrl_from_tamp(self,msg):
        self.recieved_steering = msg.steering
        self.recieved_velocity = msg.velocity
        self.received_tamp_control = True

    
    def get_control(self):
        steering = self.recieved_steering #self.tamp_control.steering
        velocity = self.recieved_velocity #self.tamp_control.velocity
        return steering,velocity         
