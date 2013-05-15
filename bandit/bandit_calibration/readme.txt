Bandit_Calibration Instructions

Author: Charles Chung Jen Wang
Interactions Lab- University of Southern California
Contact: cw_005@usc.edu
Programs: bandit_calib & bandit_calib_version1


bandit_calib is a program used to calibrate the Bandit II Robot.
bandit_calib_version1 is a program used to find the hardware limits of the Robot.


Instructions for use:

1. Run roscore

2. Run bandit_node from the bandit_driver package

***Steps 3-6 are not necessary unless the hardware limit values have been modified/deleted in bandit_calib***

3. Run bandit_calib_version1 in a seperate console window to initiate the calibration

4. In a new window type in: rosservice call calibrate_joint <joint id> <angle increment>
	<joint id> specifices the joint of interest from 0 - 18 (its values can be found in the bandit_node file)
	<angle increment> specifies the angle increments the joint will increase at until it hits its hardware limit

5. The output of the rosservice call will be:
		Joint ID: <joint id>
		MaxAngle: <maximum positive angle detected>
		MinAngle: <maximum negative angle detected>
		
		*However note that the service call is only testing each joint in one direction thus either MaxAngle or MinAngle is tested not both.

6. Record these values in the truezero function in bandit_calib.cpp for each case. Case # = Joint ID.

**************************************************************************************************************

7. Run bandit_calib from the bandit_calibration package

8. The YAML File Output will be located in the calibration package. It will be named Bandit_Calibration_File.yaml.

9. Move the Bandit_Calibration_File.yaml into the bandit_driver package and reload the bandit_node looking for the new calibration file.

10. The Bandit II should now be calibrated.



Notes:

**This program is not highly robust and may need to be ran more than once. It will not return a true zero for every joint every time.
**This program does not calibrate Joint ID: 1 (Head Pan) because there are no physical hardware limits for the head pan.
**The output YAML File is of this format:

	-
	 Joint ID:
	 Direction:
	 True zero:
	 Offset:
	 Max Angle:
	 Min Angle:

**The Direction in the YAML File refers to the directionality needed for bandit_node in order for the joints to operate symmetrically the two limbs,
not the directionality of joint calibration testing done.

**bandit_node loads the YAML File by parsing it and setting its True Zero values as the new zeros of the bandit.
