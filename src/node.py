#!/usr/bin/python

import numpy as np
import time

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, PoseStamped

class Node:

    def __init__(self):

        rospy.init_node("MAP_LOCAL")

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Starting node...")

        self.sub_tf = tf.TransformListener()
        
        self.sub_odom = rospy.Subscriber('odom_inertial', Odometry, self.odom_callback)
        
        self.pub_pose_map = rospy.Publisher('map_pose', PoseStamped, queue_size = 1)

        self.map_pose = PoseStamped()


    def run(self):

        rospy.loginfo("Starting TF broadcast")

        rospy.spin()


    def odom_callback(self, msg):

        cur_pose = PoseStamped()

        cur_pose.header.frame_id = msg.header.frame_id
        
        cur_pose.pose.position.x = msg.pose.pose.position.x
        
        cur_pose.pose.position.y = msg.pose.pose.position.y
        
        cur_pose.pose.position.z = msg.pose.pose.position.z
        
        cur_pose.pose.orientation.x = msg.pose.pose.orientation.x
        
        cur_pose.pose.orientation.y = msg.pose.pose.orientation.y
        
        cur_pose.pose.orientation.z = msg.pose.pose.orientation.z
        
        cur_pose.pose.orientation.w = msg.pose.pose.orientation.w

        if self.sub_tf.frameExists("odom") and self.sub_tf.frameExists("map_inertial"):

            self.map_pose = self.sub_tf.transformPose("map_inertial", cur_pose)

            self.pub_pose_map.publish(self.map_pose)
        

    def shutdown(self):

        pass
    
        
if __name__ == "__main__":

    try:

        node = Node()

        node.run()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo("Exiting")
