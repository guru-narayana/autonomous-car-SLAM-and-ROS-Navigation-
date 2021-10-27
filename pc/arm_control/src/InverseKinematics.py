#!/usr/bin/env python
from numpy import *
a1= 0.11
a2 = 0.11
a3 = 0.09
d = 0.03
def Ik(x,y,z,phi):
    px = sqrt(x**2 + y**2)
    py = z-d
    phi = deg2rad(phi)
    theta_0 = -arctan2(x,y) - 0.1
    wx = px - a3*cos(phi)
    wy = py - a3*sin(phi)
    delta = wx**2 + wy**2
    c2 = ( delta -a1**2 -a2**2)/(2*a1*a2)
    s2 = sqrt(1-c2**2)
    theta_2 = arctan2(s2, c2)
    c1 = ( delta + a1**2 -a2**2)/(2*a1*(sqrt(delta)))
    s1 = sqrt(1-c1**2)
    theta_1 = -(arctan2(s1, c1) + arctan2(wy,wx)) 
    theta_3 = phi-(theta_1)-theta_2
    out = [theta_0,theta_1,theta_2,theta_3]
    return out
