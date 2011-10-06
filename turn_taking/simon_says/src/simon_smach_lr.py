#!/usr/bin/env python

import roslib; roslib.load_manifest('simon_says')
import rospy
import smach
import smach_ros

import actionlib
import pose_action.msg
import sensor_msgs.msg
import wait_for_response_action.msg

from sound_play.libsoundplay import SoundClient

import simon_says.srv

import random

import yaml
from yaml import load, dump, load_all



# define state Foo
class StrikePose(smach.State):
    def __init__(self, infile):
                            
        # read poses from file
        filelist = yaml.load(file(infile,'rb').read())
        self.goal = []
        self.soundhandle = SoundClient()
        i = 0
        for filei in filelist[0]['files']:
          print filei['filename']
          filename = filei['filename']
          pose_file = open(filename,'r')
          line = pose_file.readline()
          poses = line.split()
          pose = []
          for i in poses:
            pose.append(float(i))
          names = ['left_bicep_forearm_joint', 'left_shoulder_mounting_shoulder_joint', 'left_torso_shoulder_mounting_joint', 'left_shoulder_bicep_joint', 'right_bicep_forearm_joint', 'right_shoulder_mounting_shoulder_joint', 'right_torso_shoulder_mounting_joint', 'right_shoulder_bicep_joint']
          goal = pose_action.msg.PoseGoal()
          goal.move_duration = rospy.Duration(0.5)
          goal.pose_duration = rospy.Duration(10.0)
          goal.goal_state.name = names
          goal.goal_state.position = pose
          self.goal.append( goal )
        self.client = actionlib.SimpleActionClient('pose_action_server',pose_action.msg.PoseAction)
        self.client.wait_for_server()
        rospy.loginfo('loaded file: ' + infile)
        smach.State.__init__(self,
                           outcomes=['done'],
                           output_keys=['goal_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StrikePose')
        n = random.randint(1,7)
        self.soundhandle.playWave('/home/dfseifer/turntakingwavs/simonsays'+str(n)+'.wav')
        i = random.randint(0,5)
        self.goal[i].goal_state.header.stamp = rospy.Time.now()
        userdata.goal_out = self.goal[i].goal_state
        self.client.send_goal(self.goal[i])
        rospy.sleep(1)
        return 'done'

class DemonstrationClass(smach.State):
  def __init__(self, service):
    self.pose_prob = service
    self.soundhandle = SoundClient()
    self.client = actionlib.SimpleActionClient('pose_action_server',pose_action.msg.PoseAction)
    self.client.wait_for_server()
    smach.State.__init__(self,
                         outcomes=['match','nomatch'],
                         input_keys=['dem_pose'])

  def cb(self, data):
    self.js = data

  def execute(self, userdata):
    # say text
    n = random.randint(1,4)
    file_str = '/home/dfseifer/turntakingwavs/youredoingthis'+str(n)+'.wav'
    self.soundhandle.playWave(str(file_str))
    req = pose_action.msg.PoseGoal()
    self.js = sensor_msgs.msg.JointState()
    rospy.Subscriber("output_joint_state",sensor_msgs.msg.JointState, self.cb)
    rospy.sleep(0.3)
    req.goal_state = self.js
    req.move_duration = rospy.Duration(0.5)
    req.pose_duration = rospy.Duration(10.0)
    self.client.send_goal(req)
    
    rospy.sleep(1.5)
    req.goal_state = userdata.dem_pose
    self.client.send_goal(req)
    n = random.randint(1,5)
    file_str = '/home/dfseifer/turntakingwavs/youshouldbedoingthis'+str(n)+'.wav'
    self.soundhandle.playWave(str(file_str))
    
    # for length of text + 0.5 secs, check state
    start_time = rospy.Time.now()
    while( rospy.Time.now() - start_time < rospy.Duration(5,0) and not rospy.is_shutdown()):
      rospy.sleep(0.05)
      req = simon_says.srv.ProbLRRequest()
      req.goal_state = userdata.dem_pose
      req.current_state = self.js
      try:
        resp = self.pose_prob(req)
      except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        continue
      if( resp.left_prob > 0.15 and resp.right_prob > 0.15 ):
        self.soundhandle.stopAll()
        return 'match'
    return 'nomatch'


class WaitForResponse(smach.State):
    def __init__(self, service):
        self.client = actionlib.SimpleActionClient('wait_for_response_server',wait_for_response_action.msg.WaitForResponseAction)
        self.client.wait_for_server()
        self.pose_prob = service
        smach.State.__init__(self,
                             outcomes=['nomatch','match','timeout'],
                             input_keys=['dem_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Wait For Response')
        # wait for response
        self.client.send_goal(wait_for_response_action.msg.WaitForResponseAction)
        if( not self.client.wait_for_result( rospy.Duration(5.0) ) ):
          return 'timeout'

        observed_pose = self.client.get_result()
        
        req = simon_says.srv.ProbLRRequest()
        req.goal_state = userdata.dem_pose
        req.current_state = observed_pose.pose
        try:
          resp = self.pose_prob(req)
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
               
        # does response match?
        print resp
        match = resp.left_prob > 0.2 and resp.right_prob > 0.2
        if( match ):
          return 'match'
        else:
          return 'nomatch'

class SpeechClass(smach.State):
  def __init__(self, text, num):
    self.text = text
    self.num = num
    self.soundhandle = SoundClient()
    smach.State.__init__(self,
                         outcomes=['done'])
  def execute(self, userdata):
    #rospy.loginfo("text: %s num: %d"%str(self.text)%self.num)
    # say text
    n = random.randint(1,self.num)
    file_str = '/home/dfseifer/turntakingwavs/' + self.text + str(n) + '.wav'
    rospy.loginfo("file: %s"%str(file_str))
    self.soundhandle.playWave(str(file_str))
    # for length of text + 0.5 secs, check state
    
    rospy.sleep(2)
    return 'done'

class SpeechCheckClass(smach.State):
  def __init__(self, text, num, service):
    self.text = text
    self.num = num
    self.pose_prob = service
    self.soundhandle = SoundClient()
    smach.State.__init__(self,
                         outcomes=['match','nomatch'],
                         input_keys=['dem_pose'])
  def cb(self, data):
    self.js = data

  def execute(self, userdata):
    rospy.loginfo("Text: %s"%str(self.text))
    # say text
    n = random.randint(1,self.num)
    file_str = '/home/dfseifer/turntakingwavs/'+self.text+str(n)+'.wav'
    self.soundhandle.playWave(str(file_str))
    self.js = sensor_msgs.msg.JointState()
    rospy.Subscriber("output_joint_state",sensor_msgs.msg.JointState, self.cb)
    # for length of text + 0.5 secs, check state
    start_time = rospy.Time.now()
    while( rospy.Time.now() - start_time < rospy.Duration(5,0) and not rospy.is_shutdown()):
      rospy.sleep(0.05)
      req = simon_says.srv.ProbLRRequest()
      req.goal_state = userdata.dem_pose
      req.current_state = self.js
      try:
        resp = self.pose_prob(req)
      except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        continue
      if( resp.left_prob > 0.15 and resp.right_prob > 0.15 ):
        self.soundhandle.stopAll()
        return 'match'
    return 'nomatch'


def main():
    rospy.init_node('smach_example_state_machine')
    infile = rospy.get_param('filename', 'newer_poses.yaml')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])
    sis = smach_ros.IntrospectionServer('intro',sm,'/SM_ROOT')
    sis.start()

    sm.userdata.current_goal = sensor_msgs.msg.JointState()
    sm.userdata.demonstrated_pose = sensor_msgs.msg.JointState()
    rospy.wait_for_service('pose_prob')
    pose_prob = rospy.ServiceProxy('pose_prob', simon_says.srv.ProbLR)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Strike Pose', StrikePose(infile),
                               transitions={'done':'Wait For Response'},
                               remapping={'goal_out':'demonstrated_pose'})
        smach.StateMachine.add('Positive Reinforcement', SpeechClass('goodjob',12),
                               transitions={'done': 'Strike Pose'})
        smach.StateMachine.add('Wait For Response', WaitForResponse(pose_prob),
                               transitions={'match':'Positive Reinforcement', 'nomatch':'Verify','timeout':'Timeout'},
                               remapping={'dem_pose':'demonstrated_pose'})
        smach.StateMachine.add('Timeout', SpeechClass('playwithme',5),
                               transitions={'done':'Strike Pose'})
        smach.StateMachine.add('Verify', SpeechCheckClass('areyousure',8,pose_prob),
                               transitions={'match':'Positive Reinforcement','nomatch':'General Feedback'},
                               remapping={'dem_pose':'demonstrated_pose'})
        smach.StateMachine.add('General Feedback', SpeechCheckClass('notquiteright',9,pose_prob),
                               transitions={'match':'Positive Reinforcement','nomatch':'Specific Feedback'},
                               remapping={'dem_pose':'demonstrated_pose'})
        smach.StateMachine.add('Specific Feedback', DemonstrationClass(pose_prob),
                               transitions={'match':'Positive Reinforcement','nomatch':'Reframe'},
                               remapping={'dem_pose':'demonstrated_pose'})
        smach.StateMachine.add('Reframe', SpeechClass('somethingelse',10),
                               transitions={'done': 'Strike Pose'})
  
    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()



if __name__ == '__main__':
    main()






