 Bandit Action Server (Used for Gestures)
 
 Author: Charles Chung Jen Wang
 Interactions Lab - University of Southern California
 Contact: cw_005@usc.edu
 Programs: bandit_action_server & bandit_test_client
 
 bandit_action_server is a program that serves as a running server to actuate Bandit II gestures
 bandit_test_client is a program that serves to pull out YAML Files from the bandit_gestures library and send to the bandit_action_server
 
 Instructions for Use:
 
 1. Run roscore
 
 2. Run ros_node in the bandit_driver package in a seperate console
 
 2. Run bandit_action_server in the bandit_actionlib package in a seperate console
 
 3. Run rostopic list in a seperate console
 
 4. Run rostopic echo <topic name> for the type of information desired (for example to see the feedback information run rostopic echo /bandit_actionlib/feedback)
 
 5. Run bandit_test_client in the bandit_actionlib package in a seperate console
 
 6. bandit_test_client will query the user for a filename to send to the server, the files can be found in bandit_gestures
 
 7. Once the file is sent the gesture will execute
	
		Errors: ** If the file is not found, the server side will return a warning: "File not found"
				** If the file is not correctly formatted, the server side will end in a error
				
Notes:

**the Bandit.action file can be changed to emit different feedback and results
**the library of gestures can be found in bandit_gestures file in the bandit_actionlib package
**the YAML File format for gestures is as follows:

For a Single Gestures (one YAML File)
	- Frame Num:
	  Hold Time:
	  Joints:
	  - Joint ID:
	    Joint Angle:
	  - Joint ID:
	    Joint Angle:
	    .
	    .
	    .

Frame Num indicates the frame
Hold Time indicates the time to be held in that specific frame
Joints include values of Joint ID and Joint Angles to be published
	
