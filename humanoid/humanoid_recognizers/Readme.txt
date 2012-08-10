Hello, 
A few things to study before you start tackling this code that will be helpful.  Learn the boost library particularly shared pointers.  This will be useful in other projects as the boost library is ubiquitously useful.  I would also highly recommend becoming familiar with the urdf library http://ros.informatik.uni-freiburg.de/roswiki/doc/api/urdf_interface/html/classurdf_1_1ModelInterface.html#a449837787a246a53d6f0f0579e24a3e1 and the kdl library http://www.orocos.org/wiki/orocos/kdl-wiki.  
Try to compile a few of the example programs in KDL so that you can understand what is going on.  I would highly recommend taking a day getting comfortable for each point mentioned above before touching the code otherwise you will spend alot of time struggling to contribute anything.

Also, it is important to mention that this code uses quickdev(I got it that way sorry!).  The code I am talking about from here on out is in ($find svn-directory)/usc-ros-pkg/trunk/humanoid/humanoid_recognizers/include/humanoid_recognizers/ in the file humanoid_retargeter_node.h.  Just so you know quickdev writes the code for you from a policy which is specificied in humanoid_retargeter_node.h in the macro->QUICKDEV_SPIN_FIRST().
To see your code in action simply type cd ../.. (2 directory traversals so you're in humanoid_recognizers main directory) and then type make. the executable will be in ($find svn-directory)/usc-ros-pkg/trunk/humanoid/humanoid_recognizers/bin/ named humanoid_retargeter_node

Here is a quick little overview of our full algorithm for the Kinematic solver
1. Initialization (Taken care of by me and edward -> the directory location is hardcoded for simplicity please change this)
2. Load URDF for nao (Taken care of by Edward)
3. Convert to KDL (Taken care of by Edward and me)
4. Extract Joint Limits from URDF (Taken care of by me but alot of it is hard coded for simplicity please change this)
5. Insert prismatic joints between left/right [shoulder, elbow] and [elbow, wrist] (You must do this)
6. Initialize IK solvers with subsets of above info (ie separate solvers for [torso->shoulder->elbow] and [shoulder->elbow->wrist] (You must do this there is a commented out attempt in the code)

Please keep in mind that I got the code to compile so if there are any issues (since nothing ever works the first time hahaha) it is probably something about your machine.

Once you get the initialization working you must 
At each timestep
1. Look up full state of person (via TF) (You must do this)
2. Get target position of joints on person (torso, shoulder, elbow, wrist)(You must do this)
3. Run each IK solver in order of dependencies (ie run the solver for the shoulder first, then provide the resulting pose to the solver for the elbow)(You must do this)
4. Get resulting joint angles; convert to appropriate message (probably sensor_msgs/JointState)(You must do this)
5. Publish joint angles(You must do this)

Here are some of my thoughts on how toaccomplish the rest of the described task. For inserting the prismatic joints try doing it for the urdf::model.  I am not exactly sure how one would do this so try to google it.  If that is out of the question (I have looked into it alittle and have found no fruit but this will make your life easiest)vtry doing it to the chain data type.  You might have to create 4 new segments (shoulder to prismatic joint, prismatic joint to elbow, elbow to prismatic joint, prismatic joint to wrist) for each arm and delete two connections (shoulder to elbow, elbow to wrist).  This is what I am betting will be the realistic approach.
The timestep algorithm will be fairly simple to implement and the information on how to do it can be found here: http://www.ros.org/wiki/tf.  If you have not done so already go through the tutorials.

If you have any questions please email me at matiasa123@gmail.com


