#!/usr/bin/env python
import os
from os import path
import utm
import simplekml
import numpy as np
import matplotlib.pyplot as plt
from lxml import etree 
import yaml
from coordinate_transforms import ptsFrenetToCartesian
from util import angleToContinous
#from util import angleToInterval
import re
from scipy import interpolate


def get_oval_shape_cl(R_curves,l_straight,l_connect):
    # params
    la = l_straight
    Rta = R_curves
    l_resetstraight = l_connect
    
    # section a (first straight bit)
    Na = int(la)*5
    Xa = np.linspace(0,la,Na)
    Ya = np.zeros(Na)
    
    # section b 
    Nb = np.ceil(Rta*np.pi)*5
    t = np.linspace(np.pi/2,-np.pi/2,Nb)
    Xb = Rta*np.cos(t) + Xa[-1]
    Yb = Rta*np.sin(t) - Rta + Ya[-1]
    
    # section c
    lc = la + l_resetstraight
    Nc = int(lc)*5
    Xc = np.linspace(Xb[-1],Xb[-1]-lc,Nc)
    Yc = np.zeros(Nc) + Yb[-1]
    
    # section d
    Nd = np.ceil(Rta*np.pi)*5 
    t = np.linspace(-np.pi/2,-3*np.pi/2,Nd)
    Xd = Rta*np.cos(t) + Xc[-1]
    Yd = Rta*np.sin(t) + Yc[-1]+Rta
    
    # section e
    le = l_resetstraight/2.0
    Ne = int(le)*5
    Xe = np.linspace(Xd[-1],0,Ne)
    Ye = np.zeros(Ne)
    
    # concatenate vectors
    X_cl_tmp = np.concatenate((Xa[0:-1], Xb[0:-1], Xc[0:-1], Xd[0:-1], Xe[0:-1]), axis=0)
    Y_cl_tmp = np.concatenate((Ya[0:-1], Yb[0:-1], Yc[0:-1], Yd[0:-1], Ye[0:-1]), axis=0)
    print X_cl_tmp
    
    # remove duplicate points
    threshold_dist = 0.1
    X_cl = []
    Y_cl = []
    for i in range(X_cl_tmp.size-1):
        dist = np.sqrt((X_cl_tmp[i+1]-X_cl_tmp[i])**2 + (Y_cl_tmp[i+1]-Y_cl_tmp[i])**2)
        if (dist > threshold_dist):
            X_cl.append(X_cl_tmp[i])
            Y_cl.append(Y_cl_tmp[i])        
    return np.array(X_cl), np.array(Y_cl)

