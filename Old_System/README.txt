This package is intended to be used for Synergy Motion Path Planning. 

A) Run the python joint print script on the robot computer. 
	- Follow the on screen prompts in the terminal to record the joint states
	- Specify the file name you would like to record the data in. Try to eliminate 		spaces when doing so. 
	- You will be asked if you want to record a new arm or new hand states. answering 		no to these will save the joint positions of the last updated recording 
	- When finished recording joint data there will be two files that were created, 		one that is optimized for MATLAB/Octave parsing, and another that has the 		origional Python joint position dictionaries. 

B) Copy the output files _MAT and _PY into the input folder of the Synergy Path Planning folder. 

C) After collecting the data, you can use the SynergyPathPlanning.m script to parse out the data and create a polynomial to control the joint movements. 
	-Specify the name of the input file. 
	- Specify the name of your desired output python code file
	- If you want to return to the initial position specify so when prompted. 
	- All of the time values should be entered between 0-1 as they are part of a 		normalized time scale. 



