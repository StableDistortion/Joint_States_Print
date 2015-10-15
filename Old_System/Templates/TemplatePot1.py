#!/usr/bin/env python
"""
Author: Chad Colestock
Date: 08/29/2015
Ver: 1.0
"""

#import numpy as np
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
import roslib; roslib.load_manifest('sr_ronex_examples')
from sr_ronex_msgs.msg import GeneralIOState

rospy.init_node('Synergy_move_with_pot', anonymous = True)
rospy.sleep(1.0)
hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

t = None
#told = 0;

def callback(data):
    global t
    analogue = data.analogue[0];
    t = float(int((analogue+1)))/3400
    #if t < told:
    #    t = told
   # told = t;
   # print (t)


# User input here:::

