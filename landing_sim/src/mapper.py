#!/usr/bin/env python
from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sys
import math


def callback(data):
        global xAnt
        global yAnt
        global cont

        # Create Pose Msg
        pose = PoseStamped()
        pose_star = PoseStamped()

        # GT
        pose.header.frame_id = "world"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

        # F odom
        pose_star.header.frame_id = "world"
        pose_star.pose.position.x = float(data.pose.pose.position.x)+0.02*math.sin(10*float(data.pose.pose.position.z))+0.015
        pose_star.pose.position.y = float(data.pose.pose.position.y)+0.02*math.cos(10*float(data.pose.pose.position.x))+0.01
        pose_star.pose.position.z = float(data.pose.pose.position.z)+0.02*math.sin(10*float(data.pose.pose.position.y))+0.005
        pose_star.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose_star.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose_star.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose_star.pose.orientation.w = float(data.pose.pose.orientation.w)

        # GT
        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
                pose.header.seq = path_gt.header.seq + 1
                path_gt.header.frame_id = "world"
                path_gt.header.stamp = rospy.Time.now()
                pose.header.stamp = path_gt.header.stamp
                path_gt.poses.append(pose)
                # Published the msg

        # F odom
        if (xAnt != pose_star.pose.position.x and yAnt != pose_star.pose.position.y):
                pose_star.header.seq = path_odom.header.seq + 1
                path_odom.header.frame_id = "world"
                path_odom.header.stamp = rospy.Time.now()
                pose_star.header.stamp = path_odom.header.stamp
                path_odom.poses.append(pose_star)
                # Published the msg

        cont = cont + 1

        rospy.loginfo("Hit: %i" % cont)

        
        if cont > max_append: #max_append
                path_odom.poses.pop(0)
                path_gt.poses.pop(0)

        
        pub_odom.publish(path_odom)
        pub_gt.publish(path_gt)

        xAnt = pose.pose.orientation.x
        yAnt = pose.pose.position.y
        return path_odom, path_gt


if __name__ == '__main__':
        # Initializing global variables
        global xAnt
        global yAnt
        global cont
        xAnt = 0.0
        yAnt = 0.0
        cont = 0

        # Initializing node
        rospy.init_node('gt_vs_odom_plotter')

        # Rosparams set in the launch (can ignore if running directly from bag)
        # max size of array pose msg from the path
        
        if not rospy.has_param("~max_list_append"):
                rospy.logwarn('The parameter max_list_append dont exists')
        max_append = rospy.set_param("~max_list_append", 1000) # 1000
        max_append = 1000
        if not (max_append > 0):
                rospy.logwarn('The parameter max_list_append is not correct')
                sys.exit()
        
        
        # Create the publishers
        pub_odom = rospy.Publisher('/f_odom', Path, queue_size=1)
        pub_gt = rospy.Publisher('/f_gt', Path, queue_size=1)

        path_odom = Path()
        path_gt = Path() 
        msg = Odometry()

        # Subscription to the required odom topic (edit accordingly)
        msg = rospy.Subscriber('/ground_truth/state', Odometry, callback)

        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass