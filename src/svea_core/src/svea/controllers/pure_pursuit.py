"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import rospy

class PurePursuitController(object):

    k = 0.6# look forward gain
    Lfc = 0.2 # look-ahead distance
    K_p = 0.9  #TODO speed control propotional gain
    K_i = 0.5  #TODO speed control integral gain
    K_d = 0.0  #TODO speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle
    sum_error = 0
    error_total = 0
    target_reach = False
    P = 0 
    I = 0
    D = 0
    



    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.last_time = 0.0 

    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        velocity = self.compute_velocity(state)
        return steering, velocity

    def compute_steering(self, state, target=None):
        if target is None:
            self.find_target(state)
        else:
            # allow manual setting of target
            self.target = target

        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        if state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            # stop moning if trajectory done
            # Reset control parameters for PID control
            self.P = 0.0
            self.I = 0.0
            self.D = 0.0
            return 0.0
        else:
          #return self.target_velocity
          
            # speed control
            error = self.target_velocity - state.v
            # sum of error 
            self.sum_error += error
            # time derivative first order
            get_time = rospy.get_time()
            dt = 0.01 #get_time - state.last_state_time
            self.last_time = state.time_stamp
            if error*self.error_total < 0.0: # integral anti-windup
                self.sum_error = 0
            P = self.K_p*error
            I = self.K_i*self.sum_error
            D = self.K_d*(error-self.error_total)/dt
            self.error_total = error

            control_output = P + I + D
            
            return self.target_velocity + control_output
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
        #TODO
        
        if self.target != None:
            # Calculate difference between target and actual state
            while(math.hypot(self.traj_x[-1]-state.x, self.traj_y[-1]-state.y) < 0.1):
                self.is_finished = True
        return ind
