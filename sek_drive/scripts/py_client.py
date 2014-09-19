#!/usr/bin/python

import roslib

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from tf.transformations import quaternion_from_euler
from math import radians
from nav_msgs.msg import Path

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)

        rospy.on_shutdown(self.shutdown)
        self.sent=0
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.path_sub = rospy.Subscriber('current_robot_destination', PoseWithCovarianceStamped, self.poseCallback)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_ = MoveBaseGoal()
        self.it=0
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to move base server")

        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            rospy.loginfo("wow,so spin")
            rospy.loginfo("such actionlib")
            rospy.sleep(0.5)
            if self.sent==1:
                self.move()
    def poseCallback(self, data):
        #self.goal_.header.frame_id = data.header.frame_id
        #self.goal_.goal.target_pose.pose = data.pose.pose
        self.goal_.target_pose.header.frame_id = data.header.frame_id
        self.goal_.target_pose.header.stamp = rospy.Time.now()
        self.goal_.target_pose.pose = data.pose.pose
        self.sent = 1
        
    def move(self):
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal_)

            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(10)) 
            
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                if state == GoalStatus.ABORTED:
                    rospy.loginfo("Goal Aborted")
            self.sent = 0

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
