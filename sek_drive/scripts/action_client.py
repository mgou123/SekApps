#! /usr/bin/python
#import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
import subprocess, os, signal
import geometry_msgs.msg
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path
from random import sample
from math import pow, sqrt

class RospyActionClient():
    def __init__(self):
        rospy.init_node('rospy_action_lib', anonymous=True)
        self.TIMEOUT_THRES_ = 10
        self.threshold_ = 0.3 
        self.rest_time_ = 10
        self.path_received_ = 0
        self.path_ = Path()
        self.amcl_ = PoseWithCovarianceStamped()
        self.initialpose_ = PoseWithCovarianceStamped()
        self.goal_pose_ = PoseStamped()
        self.path_length_ = 0
        self.counter_ = 0
        self.path_sent_ = 0
        self.amcl_x_ = 0
        self.amcl_y_ = 0
        self.amcl_quat_z_ = 0
        self.amcl_quat_w_ = 0
        self.init_ = 0
        self.amcl_set = 0
        self.single_goal_ = 0
        self.abort_counter_ = 0
        self.next_goal_sent_ = 0
        goal_states_ = ['PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED','REJECTED','PREEMPTING','RECALLING','RECALLED','LOST']
        self.move_base_ = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print("Before wait for server")
        self.move_base_.wait_for_server(rospy.Duration(10))
        print("Connected to move base server")
        self.lis1_ = rospy.Subscriber("sent_path", Path, self.PathCallback)
        self.lis2_ = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.AmclCallback)
        self.lis3_ = rospy.Subscriber("initial_pose", PoseWithCovarianceStamped, self.InitPoseCallback)
        self.lis4_ = rospy.Subscriber("goal_pose", PoseStamped, self.GoalPoseCallback)
        self.lis5_ = rospy.Subscriber("joy", Joy, self.CancelCallback)
        self.pub_ = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped)
        self.pub2_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
        
        while not rospy.is_shutdown():
            #print("In loop")
            if self.init_ == 1:
                printf("First If")
                self.pub2_.publish(initialpose_)
                self.init_ = 0
            
            if (self.single_goal_ == 1):
                print("In First Loop")
                self.single_goal_ = 0
                #subprocess.Popen("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &")
                print self.goal_pose_
                self.pub_.publish(self.goal_pose_)
            elif ((self.path_received_ == 1)and(self.path_length_ > 0)):
                print("In Second Loop")
                if self.counter_ == 0 :
                   
                    self.goal_ = MoveBaseGoal()
                    self.goal_.target_pose.pose = self.path_.poses[self.counter_].pose
                    self.goal_.target_pose.header.frame_id = 'map'
                    self.goal_.target_pose.header.stamp =  rospy.Time.now()
                    
                    self.goal_pose_.pose = self.path_.poses[self.counter_].pose
                    self.goal_pose_.header.frame_id = 'map'
                    self.goal_pose_.header.stamp =  rospy.Time.now()
                    
                   
                    #self.pub_.publish(self.goal_pose_)
                    if self.next_goal_sent_ == 0:
                        print("Got new target location")
                        print("Sent starting goal")
                        self.move_base_.send_goal(self.goal_)
                        self.next_goal_sent_ = 1
                    
                    state = self.move_base_.get_state()
                    print state
                    #print("Waiting to reach goal")
                    if ((state == GoalStatus.ACTIVE)or(state == GoalStatus.PENDING)): 
                        #self.move_base_.cancel_goal()
                        rospy.sleep(1)
                        self.abort_counter_ += 1
                        if self.abort_counter_ == self.TIMEOUT_THRES_:
                            print("Timed out")
                            self.path_received_ = 0
                            self.counter_ = 0
                            self.abort_counter_ = 0
                            self.next_goal_sent_ = 0
                            self.move_base_.cancel_goal()
                    elif (state == GoalStatus.SUCCEEDED):
                        #if ((state == GoalStatus.SUCCEEDED) or ((self.amcl_set == 1)and((abs(amcl_x_ - single_goal.pose.x) < self.thresold_)
                        #                                        and(abs(amcl_y_ - single_goal.pose.y) < self.thresold_)))):
                        print("Goal Reaced")
                        self.counter_ += 1
                        self.abort_counter_ += 1
                        self.counter_ = 0
                        self.abort_counter_ = 0
                    else:
                        print("Goal Failed with error code: "+str(goal_states_[state]))
                        self.path_received_ = 0
                        self.counter_ = 0
                        self.next_goal_sent_ = 0
                        self.abort_counter_ = 0
                else :
                    if self.counter_ == self.path_length :#_ -1 :
                        print("Reached end of path")
                        self.path_received_ = 0
                        self.counter_ = 0
                        self.next_goal_sent_ = 0
                        self.abort_counter_ =0
                    else :
                        self.goal_ = MoveBaseGoal()
                        self.goal_.target_pose.pose = self.path_.poses[self.counter_].pose
                        self.goal_.target_pose.header.frame_id = 'map'
                        self.goal_.target_pose.header.stamp =  rospy.Time.now()
                        
                        self.goal_pose_.pose = self.path_.poses[self.counter_].pose
                        self.goal_pose_.header.frame_id = 'map'
                        self.goal_pose_.header.stamp =  rospy.Time.now()
                        
                        print("Got new target location")
                        
                        #self.pub_.publish(self.single_goal_)
                        if self.next_goal_sent_ == 0:
                            self.move_base_.send_goal(self.goal_)
                            self.next_goal_sent_ = 1
                            self.counter_ = 0
                            self.abort_counter_ = 0
                        
                        state = self.move_base.get_state()
                        print state
                        if ((state == GoalStatus.ACTIVE)or(state == GoalStatus.PENDING)): 
                        #self.move_base_.cancel_goal()
                            rospy.sleep(1)
                            self.abort_counter_ += 1
                            if self.abort_counter_ == self.TIMEOUT_THRES_:
                                print("Timed out")
                                self.path_received_ = 0
                                self.counter_ = 0
                                self.abort_counter_ = 0
                                self.next_goal_sent_ = 0
                                self.move_base_.cancel_goal()
                        elif (state == GoalStatus.SUCCEEDED):
                            #if ((state == GoalStatus.SUCCEEDED) or ((self.amcl_set == 1)and((abs(amcl_x_ - single_goal.pose.x) < self.thresold_)
                            #                                        and(abs(amcl_y_ - single_goal.pose.y) < self.thresold_)))):
                            print("Goal Reached")
                            self.counter_ += 1
                            self.abort_counter_ = 0
                            self.counter_ = 0
                            self.abort_counter_ = 0
                        else:
                            print("Goal Failed with error code: "+str(goal_states_[state]))
                            self.path_received_ = 0
                            self.counter_ = 0
                            self.next_goal_sent_ = 0
                            self.abort_counter_ = 0
            else:
                print("Default case")
                rospy.sleep(0.5)
                
    def PathCallback(self, data):
        print("In PathCallback")
        if (data.header.frame_id == 'stop'):
            self.path_received_ = 0
            subprocess.Popen("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &")
            self.counter_ = 0
        else:
            #subprocess.Popen("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &")
            self.path_received_ = 1
            self.single_goal = 0
            self.path_ = data
            print self.path_
            self.path_length_ = len(data.poses)
        print("Got path of size : " + str(self.path_length_))
        
    def AmclCallback(self, data):
        print("In AmclCallback")
        self.amcl_set_ = 1
        self.amcl_x_ = data.pose.pose.position.x
        self.amcl_y_ = data.pose.pose.position.y
        self.amcl_quat_z_ = data.pose.pose.orientation.z
        self.amcl_quat_w_ = data.pose.pose.orientation.w
    
    def InitPoseCallback(self, data):
        print("In InitPoseCallback")
        self.init_ = 1
        self.initialpose_.pose = data.pose
        self.initialpose_.header.frame_id = "/map"
        self.initialpose_.header.stamp = rospy.Time.now()
    
    def GoalPoseCallback(self, data):
        print("GoalPoseCallback")
        #subprocess.Popen("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &")
        self.single_goal_ = 1
        self.path_received_ = 0
        self.goal_pose_.pose = data.pose
        self.goal_pose_.header.frame_id = "/map"
        self.goal_pose_.header.stamp = rospy.Time.now()
        #print self.goal_pose_
        
    def CancelCallback(self, data):
        print("CancelCallback")
        if data.buttons[1] == 1:
            self.path_received = 0
            self.counter = 0
            self.single_goal = 0 
            
    def shutdown(self):
        print("Stopping the robot ...")
        self.move_base_.cancel_goal()
        rospy.sleep(2)
        
if __name__ == '__main__':
    try:
        print ("In main")
        RospyActionClient()
        print ("Before spin")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("AMCL navigation test finished")
        
