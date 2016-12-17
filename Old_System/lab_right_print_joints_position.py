#!/usr/bin/env python

"""
Allows for a guided process to print out consecutive joint states of the UR10. Useful for
building synergy path movements or multistage motions in general.

Author: Thomas Colestock
Date: 08/29/2015
Ver: 1.0.0
"""

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("print_left_joints_position", anonymous=True)
hand_commander = SrHandCommander()

__author__ = 'Thomas Colestock'
print("Joints positions")

hand_keyID = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1',
              'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_THJ1',
              'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2']

arm_keyID = ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 'ra_wrist_1_joint',
             'ra_wrist_2_joint', 'ra_wrist_3_joint']

filePrefix = raw_input("Input Desired File Name Without Suffix or Spaces (.txt): ")
print("Two files will be created: \n" + filePrefix + "_PY.txt\n" + filePrefix + "_MAT.txt\n")
print("_PY.txt creates a Python Dictionary Reference and _MAT.txt creates a file easily parsed by MATLAB \n")
print("!! REMEMBER TO MOVE FILES TO A NEW LOCATION WHEN FINISHED OR THEY WILL BE OVERWRITTEN !! \n\n")

FileObj = open(filePrefix + "_MAT.txt", 'a')
FileObj2 = open(filePrefix + "_PY.txt", 'a')

FileObj.write("Hand Joints are in the order of: \n")
for i in range(len(hand_keyID)):
    FileObj.write(str(hand_keyID[i]) + "\n")
FileObj.write("\n")

FileObj.write("Arm Joints are in the order of: \n")
for i in range(len(arm_keyID)):
    FileObj.write(str(arm_keyID[i]) + "\n")
FileObj.write("************************* \n\n")

FileObj2.write("# Joint Position Dictionaries for Arm and Hand:\n\n")

flag = 1
new_hand = 'y'
new_arm = 'y'

while flag == 1:
    input_val = raw_input("Specify Current Joint State Name: ")

    all_joints_state = hand_commander.get_joints_position()
    hand_joints_state = {k: v for k, v in all_joints_state.items() if k.startswith("rh_")}
    arm_joints_state = {k: v for k, v in all_joints_state.items() if k.startswith("ra_")}

    print("<" + input_val + ">\n")
    print("Hand joints position \n " + str(hand_joints_state) + "\n")
    print("Arm joints position \n " + str(arm_joints_state) + "\n")

    # Write data to files
    # Write to _MAT file:
    FileObj.write("<" + input_val + ">\n")
    FileObj.write("Hand joints position\n")
    # Write Hand joints in _MAT
    if new_hand == 'y':
        for i in range(len(hand_keyID)):
            FileObj.write(str(hand_joints_state[hand_keyID[i]]) + "\n")
        FileObj.write("HEOB\n\n")

    if new_hand == 'n':
        for i in range(len(hand_keyID)):
            FileObj.write(str(old_hand_joints_state[hand_keyID[i]]) + "\n")
        FileObj.write("HEOB\n\n")

    # Write Arm joints in _MAT
    FileObj.write("Arm joints position\n")
    if new_arm == 'y':
        for i in range(len(arm_keyID)):
            FileObj.write(str(arm_joints_state[arm_keyID[i]]) + "\n")
        FileObj.write("AEOB\n\n\n")

    if new_arm == 'n':
        for i in range(len(arm_keyID)):
            FileObj.write(str(old_arm_joints_state[arm_keyID[i]]) + "\n")
        FileObj.write("AEOB\n\n\n")

    # Write to _PY file:
    # Write Hand joints to _PY
    if new_hand == 'y':
        FileObj2.write(input_val + "_Hand = " + str(hand_joints_state) + "\n\n")
    if new_hand == 'n':
        FileObj2.write(input_val + "_Hand = " + str(old_hand_joints_state) + "\n\n")

    # Write Arm joints to _PY
    if new_arm == 'y':
        FileObj2.write(input_val + "_Arm = " + str(arm_joints_state) + "\n\n")
    if new_arm == 'n':
        FileObj2.write(input_val + "_Arm = " + str(old_arm_joints_state) + "\n\n")

    new_hand_old_flag = new_hand
    new_arm_old_flag = new_arm

    should_run = raw_input("Would you like to record another joint state? [y/n] ")
    if should_run == 'n':
        flag = 0
        break
    else:
        new_hand = raw_input("Record a new set of Hand Joints: [y/n] ")
        new_arm = raw_input("Record a new set of Arm Joints: [y/n] ")

    if new_hand != new_hand_old_flag:
        old_hand_joints_state = hand_joints_state
    if new_arm != new_arm_old_flag:
        old_arm_joints_state = arm_joints_state

FileObj.close()
FileObj2.close()
