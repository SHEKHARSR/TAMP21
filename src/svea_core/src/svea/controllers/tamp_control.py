import rospy
import math
from svea_msgs.msg import lli_ctrl
from svea_msgs.msg import lli_emergency
from svea_msgs.msg import tamp_control

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
        #self.control_update_sub = rospy.Subscriber('/lli/ctrl_request', lli_ctrl, self.callback_ctrl_interface)
        self.control_update_sub = rospy.Subscriber('/Control_signal', tamp_control, self.callback_ctrl_from_tamp)
        self.tamp_control = tamp_control()
        self.received_tamp_control = False 
        

    """def callback_ctrl_interface(self,lli_ctrl):
        self.get_steering = lli_ctrl.steering
        self.get_velocity = lli_ctrl.velocity"""
    
    def callback_ctrl_from_tamp(self,tamp_contol):
        self.get_steering = tamp_control.steering
        self.get_velocity = tamp_control.velocity
        self.received_tamp_control = True

    
    def get_control(self):
        steering = tamp_control.steering
        velocity = tamp_control.velocity
        return steering,velocity         
    def find_target(self, state):
        ind = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)

    def _calc_target_index(self, state):
        # search nearest point index
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * state.v + self.Lfc

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition       
        if self.target != None:
            # Calculate difference between target and actual state
            if(math.hypot(self.traj_x[-1]-state.x, self.traj_y[-1]-state.y) < 0.1):
                self.is_finished = True
        return ind