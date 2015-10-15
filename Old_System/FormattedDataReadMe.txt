*
* This is a clean version of the joint data for the EMG/EEG Pick N Place synergy test 1
*
* The format will be used in an attempt to create an easily read file for MATLAB
*
* Hand Joints will be labeled as " Hand joints position " and will follow this order of values: 
*
*
*1 rh_FFJ1 
*
*2 rh_FFJ2
*
*3 rh_FFJ3 
*
*4 rh_FFJ4 
*
*5 rh_THJ4 
*
*6 rh_THJ5 
*
*7 rh_THJ1 
*
*8 rh_THJ2 
*
*9 rh_THJ3 
*
*10 rh_LFJ2 
*
*11 rh_LFJ3 
*
*12 rh_LFJ1 
*
*13 rh_LFJ4 
*
*14 rh_LFJ5 
*
*15 rh_RFJ4 
*
*16 rh_RFJ1 
*
*17 rh_RFJ2 
*
*18 rh_RFJ3 
*
*19 rh_MFJ1 
*
*20 rh_MFJ3 
*
*21 rh_MFJ2 
*
*22 rh_MFJ4 
*
*23 rh_WRJ2 
*
*24 rh_WRJ1 
*
*
* Arm Joints will be labeled as " Arm joint position " and will follow this order of values: 
*
*
* ra_shoulder_pan_joint
*
* ra_elbow_joint 
*
* ra_wrist_1_joint 
*
* ra_shoulder_lift_joint 
*
* ra_wrist_3_joint 
*
* ra_wrist_2_joint
*
* The listed values will not have lines between them in the printout, they are here for clarity in reading only. 
*
* The MATLAB code will parse out the data and create a nxm matrix where n = the number of joints and m = the * * number of data sets for the hand or arm. It will also put the joints in proper order number wise with each 
* digit in the order of FF, MF, RF, LF, Th for the hand and shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, * wrist_3 for the arm. 
* 
* Any comments that are to be manually inputted into this file should be preceeded with a ' * '. 
*
* To keep track of desired movement names preceede them with a '<' followed by a '>' like: <Home>
*
***********************************************************************************************