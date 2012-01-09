#!/usr/bin/env python

import roslib; roslib.load_manifest('simon_says')
import rospy
import smach
import smach_ros

import actionlib
import pose_action.msg

import yaml
from yaml import load, dump, load_all

# define state Foo
class StrikePose(smach.State):
    def __init__(self, infile):
        smach.State.__init__(self, outcomes=['nomatch'])




    def execute(self, userdata):
        rospy.loginfo('Executing state StrikePose')
        rospy.sleep(10.)
        return 'nomatch'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['match','nomatch'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(10.)
        return 'nomatch'

def main():
    rospy.init_node('smach_example_state_machine')
    infile = rospy.get_param('filename', 'newer_poses.yaml')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])
    sis = smach_ros.IntrospectionServer('intro',sm,'/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Strike Pose', StrikePose(infile),
                               transitions={'nomatch':'BAR'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'match':'done', 'nomatch':'done'})

    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()






