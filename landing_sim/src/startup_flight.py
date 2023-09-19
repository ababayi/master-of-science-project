#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import hector_uav_msgs
from hector_uav_msgs.msg import PoseAction
from visualization_msgs.msg import Marker
from hector_uav_msgs.msg import PoseActionGoal
from geometry_msgs.msg import Twist

# Define The Goal
x_goal = 0
y_goal = 0
z_goal = 10
    

if __name__ == '__main__':

  ''' Notes:
    
  Node for startup flight
    
  '''
  # Initiate Node
  rospy.init_node('startup_flying_node')

  # Create a client for runnig the hector quad-copter
  client = actionlib.SimpleActionClient('/action/pose', PoseAction)
  rospy.loginfo("Waiting for Server")
  client.wait_for_server()
  rospy.loginfo("Server Started")
  
  vel_goal_msg = Twist() # velosiy and goal message for gzbo
  
  # Create publisher for hector (/action/pose/goal) and gazebeo (/cmd_vel)
  pub = rospy.Publisher('/action/pose/goal', PoseActionGoal, queue_size=10)
  pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

  # Define the goal for flight
  vel_goal_msg = Twist() # velosiy and goal message for gzbo
  goal_pose = PoseActionGoal()
  goal_pose.header.seq= waypoint_index = 0
  goal_pose.header.frame_id= 'world'
  goal_pose.goal.target_pose.header.frame_id = 'world'
  goal_pose.goal.target_pose.pose.position.x  = x_goal
  goal_pose.goal.target_pose.pose.position.y  = y_goal
  goal_pose.goal.target_pose.pose.position.z  = z_goal

  # Publish and fly
  rospy.loginfo(goal_pose)
  rospy.loginfo(goal_pose.goal_id.stamp)
  pub.publish(goal_pose)
