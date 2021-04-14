#!/usr/bin/env python

# Descrition: Friction estimation

# subscribes:
# state (incl s and rho)
# planned traj (incl s and mu)

# publishes: 
# friction estimate (todo determine format and how to incorporate in pathlocal/global)

import time
import numpy as np

#import matplotlib
#matplotlib.use('TkAgg')
#import matplotlib.pyplot as plt

import matplotlib.pyplot as plt

import rospy
from common.msg import State
from common.msg import Trajectory
from common.msg import MuEst

class FrictionEstimation:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('friction_estimation', anonymous=True)
        self.rate = rospy.Rate(10)  # approx 10hz update rate of live plot

        # params
        self.N_buffer = rospy.get_param('/N_buffer_fe')
        self.N_est = 100
        self.ds_est = 1.0
        self.idx_lh = rospy.get_param('/idx_lh')
        self.mu_mean_nominal = rospy.get_param('/mu_nominal')
        self.mu_sigma_nominal = rospy.get_param('/mu_sigma_nominal')
        self.Ff_util = rospy.get_param('/Ff_util')
        self.Fr_util = rospy.get_param('/Fr_util')

        # subs & pubs        
        self.state_sub = rospy.Subscriber("/state", State, self.state_callback)
        self.state = State()
        self.received_state = False
        
        self.trajstar_sub = rospy.Subscriber("/trajstar", Trajectory, self.trajstar_callback)
        self.trajstar = Trajectory()
        self.received_trajstar = False        
        
        self.mu_est_pub = rospy.Publisher("/mu_est", MuEst, queue_size=1)
        self.mu_est = MuEst()
        self.mu_est.s = np.zeros(self.N_est)
        self.mu_est.mu_est_mean = np.zeros(self.N_est)
        self.mu_est.mu_est_sigma = np.zeros(self.N_est)
        
        
        # init node vars
        self.s_planned_buffer = np.zeros(self.N_buffer)
        self.rhof_planned_buffer = np.zeros(self.N_buffer)
        self.rhor_planned_buffer = np.zeros(self.N_buffer)
        
        self.s_vec = np.zeros(int(self.N_est/2.))
        self.rhof_perceived_vec = np.zeros(int(self.N_est/2.))
        self.rhof_planned_vec = np.zeros(int(self.N_est/2.))
        self.count_state_callback = np.longlong(0)

        self.cov_last = np.array([[],[]])
        
        #self.rhof_planned = np.zeros(self.N_buffer)
        
        # init live plot
        self.do_live_plot = True
        if self.do_live_plot:
            fig = plt.figure(num=None, figsize=(24, 12), dpi=80, facecolor='w', edgecolor='k')
            ax = fig.add_subplot(111)
            line_objs = ax.plot(np.arange(self.N_buffer),np.zeros(self.N_buffer),'b.',
                                 np.arange(self.N_buffer),np.zeros(self.N_buffer),'r.',
                                 np.arange(self.N_est),np.zeros(self.N_est),'k-', 
                                 np.arange(self.N_est),np.zeros(self.N_est),'k--', 
                                 np.arange(self.N_est),np.zeros(self.N_est),'k--',ms = 20,lw = 5)

            rho_perceived_ln = line_objs[0]
            rho_planned_ln = line_objs[1]
            mu_est_mean_ln = line_objs[2]
            conf_intv_ub_ln = line_objs[3]
            conf_intv_lb_ln = line_objs[4]
            
            ax.legend(['perceived','planned', 'mu est mean', 'conf_itv'])
            ax.set_xlabel('s (m)')
            ax.set_ylabel('mu')
            
            ax.set_ylim([-0.1,1.1])
            plt.ion()
            plt.show()  

        
        while(not self.received_state):
            rospy.loginfo_throttle(1, "friction_est: waiting for state")
            rospy.sleep(0.1)

        while(not self.received_trajstar):
            rospy.loginfo_throttle(1, "friction_est: waiting for trajectory")
            rospy.sleep(0.1)

        rospy.loginfo("friction_est: started") 
        
        # main loop - only live plot
        t = time.time()
        while not rospy.is_shutdown():
            
            # update measurement
            self.s_train = np.array([])
            self.mu_train = np.array([])
            mgn = 0.2
            for i in range(self.s_vec.size):
                if (self.rhof_perceived_vec[i] < self.rhof_planned_vec[i] - mgn):
                    self.s_train = np.append(self.s_train,self.s_vec[i])
                    self.mu_train = np.append(self.mu_train,self.rhof_perceived_vec[i])
                                
            # GP regression
            self.mu_est.s = np.linspace(self.state.s-0.5*self.N_est*self.ds_est, self.state.s+0.5*self.N_est*self.ds_est, self.N_est)             
            if (self.s_train.size > 0):
                s_train_cv = np.reshape(self.s_train,(self.s_train.size,1))
                mu_train_cv = np.reshape(self.mu_train,(self.mu_train.size,1))
                
                s_est_cv = np.reshape(self.mu_est.s,(self.mu_est.s.size,1))
                #mean, sigma, K = self.posterior(s_train_cv,mu_train_cv,s_est_cv,500.0,0.01,self.mu_sigma_nominal,self.mu_mean_nominal)
                mean, cov = self.posterior_(X_s = s_est_cv, 
                                            X_train = s_train_cv, 
                                            Y_train = mu_train_cv, 
                                            l = 25.0, 
                                            sigma_f = self.mu_sigma_nominal, 
                                            sigma_y = 0.1,
                                            prior_mean = self.mu_mean_nominal,
                                            prior_cov = self.cov_last)
                
                rospy.logwarn("friction_est: diag(cov) = " + str(np.diag(cov)))
                
                sigma = np.sqrt(np.diag(cov))
                
                
                self.mu_est.mu_est_mean = mean.flatten()
                self.mu_est.mu_est_sigma = sigma.flatten()
                
#                # shift and store covariance matrix
#                self.cov_last = cov
#                #roll rows down
#                self.cov_last = np.roll(self.cov_last, 1, axis=0)
#                self.cov_last[0,:] = np.zeros(self.N_est)
#                # roll colums rgt
#                self.cov_last = np.roll(self.cov_last, 1, axis=1)
#                self.cov_last[:,0] = np.zeros(self.N_est)
#                # set variance at [0,0]
#                self.cov_last[0,0] = self.mu_sigma_nominal**2

                
            else:
                self.mu_est.mu_est_mean = self.mu_mean_nominal*np.ones(self.N_est) 
                self.mu_est.mu_est_sigma = self.mu_sigma_nominal*np.ones(self.N_est)
                self.cov_last = np.array([[],[]])
                 
            self.mu_est.header.stamp = rospy.Time.now()
            self.mu_est_pub.publish(self.mu_est)

            
            # update plot
            if self.do_live_plot:
                x_axis_buf = self.s_vec-self.state.s
                x_axis_est = self.mu_est.s-self.state.s
                
                rho_perceived_ln.set_xdata(x_axis_buf)
                rho_perceived_ln.set_ydata(self.rhof_perceived_vec)
                
                rho_planned_ln.set_xdata(x_axis_buf) 
                rho_planned_ln.set_ydata(self.rhof_planned_vec) 
                
                #rospy.loginfo("friction_est: len x: " + str(len(x_axis)))
                #y_axis = np.array(self.mu_est.mu_est_mean) + 1.96*np.array(self.mu_est.mu_est_sigma)
                #rospy.loginfo("friction_est: max y: " + str(np.max(y_axis)))
                
                mu_est_mean_ln.set_xdata(x_axis_est)
                mu_est_mean_ln.set_ydata(self.mu_est.mu_est_mean )
                
                conf_intv_ub_ln.set_xdata(x_axis_est)
                conf_intv_ub_ln.set_ydata(self.mu_est.mu_est_mean + 1.96*np.array(self.mu_est.mu_est_sigma))
                
                conf_intv_lb_ln.set_xdata(x_axis_est)
                conf_intv_lb_ln.set_ydata(self.mu_est.mu_est_mean - 1.96*np.array(self.mu_est.mu_est_sigma))
                
                plt.pause(0.0001)
                ax.set_xlim([-60, 60])
                
                rospy.loginfo("friction_est: looptime plot update: " + str(time.time() - t))
                t = time.time()
                
            self.rate.sleep()
    
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True
        
        if(self.state.vx > 5.0): # avoid issues at low speed (also we wont have useful data at very low speeds)
            
            s_lh = self.trajstar.s[self.idx_lh]
            rhof_lh = np.sqrt(self.trajstar.Fxf[self.idx_lh]**2 + self.trajstar.Fyf[self.idx_lh]**2)/self.trajstar.Fzf[self.idx_lh]
            #rhor_lh = np.sqrt(self.trajstar.Fxr[self.idx_lh]**2 + self.trajstar.Fyr[self.idx_lh]**2)/self.trajstar.Fzr[self.idx_lh]
            self.s_planned_buffer = np.roll(self.s_planned_buffer,-1)
            self.s_planned_buffer[-1] = s_lh
            self.rhof_planned_buffer = np.roll(self.rhof_planned_buffer,-1)
            self.rhof_planned_buffer[-1] = rhof_lh*self.Ff_util # compensate
            
            #self.rhor_planned_buffer = np.roll(self.rhor_planned_buffer,1)
            #self.rhor_planned_buffer[0] = rhor_lh
            
            # interp if buffers are filled
            #if (np.count_nonzero(self.s_planned_buffer) == self.s_planned_buffer.size):
            if (self.s_planned_buffer.min() < self.state.s and self.s_planned_buffer.max() > self.state.s):    
                rhof_planned = np.interp(self.state.s, self.s_planned_buffer, self.rhof_planned_buffer)
            else:
                rhof_planned = float('Nan')
                rospy.logerr("friction est: self.state.s not in interval, state.s = " + str(self.state.s) + ", s_planned_buffer[0] = " + str(self.s_planned_buffer[0]) + ", s_planned_buffer[-1] = " + str(self.s_planned_buffer[-1]))

            # update rho vectors 
            #if (self.count_state_callback%10 == 0):
            if (self.state.s > self.s_vec[-1] + self.ds_est):
                self.s_vec = np.roll(self.s_vec,-1)
                self.s_vec[-1] = self.state.s
    
                self.rhof_perceived_vec = np.roll(self.rhof_perceived_vec,-1)
                self.rhof_perceived_vec[-1] = self.state.rhof
                
                self.rhof_planned_vec = np.roll(self.rhof_planned_vec,-1)
                self.rhof_planned_vec[-1] = rhof_planned
           
                # cut tail of vectors
    #            idx = self.s_vec < self.state.s - 0.5*self.N_est*self.ds_est
    #            self.s_vec = self.s_vec[idx]
    #            self.rhof_perceived_vec = self.rhof_perceived_vec[idx]
    #            self.rhof_planned_vec = self.rhof_planned_vec[idx]

        self.count_state_callback += 1


#    def posterior(self, Xtrain, ytrain, Xtest, l2=0.1, noise_var_train=1e-6,sigma_f=1.0,yoffset=0.8):
#        #ytrain_zeromean = ytrain-np.mean(ytrain)
#        ytrain_shifted = ytrain-yoffset
#        # compute the mean at our test points.
#        Ntrain = len(Xtrain)
#        K = self.kernel(Xtrain, Xtrain, l2,sigma_f)
#        L = np.linalg.cholesky(K + noise_var_train*np.eye(Ntrain))
#        Lk = np.linalg.solve(L, self.kernel(Xtrain, Xtest, l2,sigma_f))
#        #mean = np.dot(Lk.T, np.linalg.solve(L, ytrain_zeromean)) + np.mean(ytrain)
#        mean = np.dot(Lk.T, np.linalg.solve(L, ytrain_shifted)) + yoffset
#        # compute the variance at our test points.
#        K_ = self.kernel(Xtest, Xtest, l2,sigma_f)
#        
#        sd = np.sqrt(np.diag(K_) - np.sum(Lk**2, axis=0))
#        return (mean, sd, K)


    def posterior_(self, X_s, X_train, Y_train, l=1.0, sigma_f=1.0, sigma_y=1e-8,prior_mean=0.0,prior_cov=np.array([[],[]])):
        """        
        Args:
            X_s: New input locations (n x d).
            X_train: Training locations (m x d).
            Y_train: Training targets (m x 1).
            l: Kernel length parameter.
            sigma_f: Kernel vertical variation parameter.
            sigma_y: Noise parameter.
        
        Returns:
            Posterior mean vector (n x d) and covariance matrix (n x n).
        """
        Y_train_shifted = Y_train - prior_mean
        
        K = self.kernel_(X_train, X_train, l, sigma_f) + sigma_y**2 * np.eye(len(X_train))
        K_inv = np.linalg.pinv(K)
        K_s = self.kernel_(X_train, X_s, l, sigma_f)
        
        if (prior_cov.size == 0):
            K_ss = self.kernel_(X_s, X_s, l, sigma_f) + sigma_y**2 * np.eye(len(X_s))
        else:
            K_ss = prior_cov.copy()
        
        mean_s = K_s.T.dot(K_inv).dot(Y_train_shifted) + prior_mean
        cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)

        return mean_s, cov_s



#    def kernel(self,x, y, l2=0.1, sigma_f=1.0):
#        sqdist = np.sum(x**2,1).reshape(-1,1) + \
#                 np.sum(y**2,1) - 2*np.dot(x, y.T)
#        return sigma_f**2 * np.exp(-.5 * (1/l2) * sqdist)                


    def kernel_(self,X1, X2, l=1.0, sigma_f=1.0):
        """
        Isotropic squared exponential kernel.
        
        Args:
            X1: Array of m points (m x d).
            X2: Array of n points (n x d).
    
        Returns:
            (m x n) matrix.
        """
        sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
        return sigma_f**2 * np.exp(-0.5 / l**2 * sqdist)

                
    def trajstar_callback(self, msg):
        self.trajstar = msg
        self.received_trajstar = True


if __name__ == '__main__':
    fe = FrictionEstimation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
