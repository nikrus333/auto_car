#!/usr/bin/env python3

import os
import rospy
import time
import cv2


class PlanWay():
    def __init__(self):
        self.base_dot = [0 , 0]
        self.end_dot = [10 , 5]
    
    def curve_baze(self,t):
        B0 = 1
        B1 = 1
        B2 = 1
        B3 = 1
        B4 = 1
        J_3_0 = (1 - t)**3
        J_3_1 = 3 * t * (1 - t)**2
        J_3_2 = 3 * t**2 * (1 - t)
        J_3_3 = t**3
        P = B0 * J_3_0 + B1 * J_3_1 + B2 * J_3_2 + B3 * J_3_3
        P_dif = (-3 * (1 - t)**2 * B0) + (B1 * (3 * (1 - t)**2 - 6 * t * (1 - t))) + (B2 *  (6 * t * (1 - t) - 3 * t**2)) + B3 * 3 * t**2
            

    def very_base(self):
        pass
    def visual_traectory(self):

class GetWay():
    pass

