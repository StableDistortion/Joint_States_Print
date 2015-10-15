# User input ends here::::
# ==============================================================================

#Author: Chad Colestock
#Date: 08/29/2015
#Ver: 1.0

hand_joint_names = ['rh_FFJ1','rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4','rh_MFJ1','rh_MFJ2','rh_MFJ3','rh_MFJ4','rh_RFJ1','rh_RFJ2','rh_RFJ3','rh_RFJ4','rh_LFJ1','rh_LFJ2','rh_LFJ3','rh_LFJ4','rh_LFJ5','rh_THJ1','rh_THJ2','rh_THJ3','rh_THJ4','rh_THJ5','rh_WRJ1','rh_WRJ2'];
arm_joint_names = ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 'ra_wrist_1_joint', 'ra_wrist_2_joint', 'ra_wrist_3_joint'];

move_dict_arm = {};
move_dict_hand = {};


#move hand to start position
rospy.loginfo("Move to hand_start position")
hand_commander.move_to_joint_value_target_unsafe(hand_start, 4, True)
arm_commander.move_to_joint_value_target_unsafe(arm_start, 4, True)


rospy.sleep(1.0)
#run loop at 50Hz
r = rospy.Rate(50)


while not rospy.is_shutdown():
    for i in range(len(movements_Arm)):
        current_move_dict_arm = movements_Arm[i];
        current_move_dict_hand = movements_Hand[i];
        tlist = times[i];
        print('Next movement: ' + movements[i])

        rospy.Subscriber("/ronex/general_io/1409580174/state", GeneralIOState, callback);
        print(t)
        if tlist[0] == 0:
            print('Please reduce all excitation to 0 to enter movement phase \n')
            while t > 0.01:
                print('Current excitation is: ')
                print(t)
                print('\nPlease reduce to 0 \n' )
                rospy.Subscriber("/ronex/general_io/1409580174/state", GeneralIOState, callback);
                rospy.sleep(.5)
            print('Starting motion: ' + movements[i])
            t = 0.01;

        while t < tlist[1]: # t is the normalized excitation value from pot/emg/eeg

            for key in arm_joint_names:
                polyCoef = current_move_dict_arm[key];
                jointPos = polyCoef[0]*t**3 + polyCoef[1]*t**2 + polyCoef[2]*t + polyCoef[3];
                move_dict_arm[key] = jointPos;
            for key in hand_joint_names:
                polyCoef = current_move_dict_hand[key];
                jointPos = polyCoef[0]*t**3 + polyCoef[1]*t**2 + polyCoef[2]*t + polyCoef[3];
                move_dict_hand[key] = jointPos;

            # print(move_dict_arm )
            #print('\n')
            #print(move_dict_hand)
            #print('\n')
            arm_commander.move_to_joint_value_target_unsafe(move_dict_arm,wait=False)
            hand_commander.move_to_joint_value_target_unsafe(move_dict_hand,wait=False)

            told = t; # to keep moving forward only NOT CURRENTLY WORKING
            #print(told)
            #print('\n')
            rospy.Subscriber("/ronex/general_io/1409580174/state", GeneralIOState, callback);
            #print(data)
            #analogue = data.analogue[0];
            #t = float(int((analogue+1)/3400))
            if t < told: # keep moving forward only
                t = told;
            if t < 0.01:
                t = 0.01;
            #rospy.spin()
            r.sleep()



