#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Antons Rebguns. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
#roslib.load_manifest('chr_6d_imu')
import rospy

from chr_6d import CHR6D
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from tf import *
import tf
#import tf2_ros

class CHR6DNode():
    def __init__(self):
        port_name = rospy.get_param('~port_name', '/dev/ttyUSB0')
        frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.mode = rospy.get_param('~mode', 'polled')
        self.ignore_pitch_roll = rospy.get_param('~ignore_pitch_roll', True)
        self.data_rate = rospy.get_param('~data_rate', 150)
        
        self.imu = CHR6D(port_name)
        if self.mode == 'streaming':
            self.imu.set_broadcast_mode(self.data_rate)
        elif self.mode == 'polled':
            self.imu.set_silent_mode()
        else:
            rospy.logwarn('Unrecognized IMU node, setting to polled')
            self.imu.set_silent_mode()
        
        rospy.sleep(0.1)
        
        self.imu.zero_rate_gyros()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id
        
        self.imu_msg.linear_acceleration_covariance[0] = 0
        self.imu_msg.linear_acceleration_covariance[4] = 0
        self.imu_msg.linear_acceleration_covariance[8] = 0
       
        self.imu_msg.angular_velocity_covariance[0] = 0
        self.imu_msg.angular_velocity_covariance[4] = 0
        self.imu_msg.angular_velocity_covariance[8] = 0
       
        self.imu_msg.orientation_covariance[0] = 0
        self.imu_msg.orientation_covariance[4] = 0
        self.imu_msg.orientation_covariance[8] = 0
       
        self.imu_data_pub = rospy.Publisher('imu/data', Imu)

    def publish_data(self):
        r = rospy.Rate(self.data_rate)
        
        while not rospy.is_shutdown():
            if self.mode == 'streaming':
                data = self.imu_read_accel_angrate_orientation()
            else:
                data = self.imu.get_data()
            if not data: continue
            
            if self.ignore_pitch_roll:
                data['roll'] = 0.0
                data['pitch'] = 0.0
                
            ori = quaternion_from_euler(data['roll'], data['pitch'], 0)
            
            self.imu_msg.orientation.x = ori[1]
            self.imu_msg.orientation.y = ori[0]
            self.imu_msg.orientation.z = -ori[2]
            self.imu_msg.orientation.w = ori[3]
            
            self.imu_msg.angular_velocity.x = 0
            self.imu_msg.angular_velocity.y = 0
            self.imu_msg.angular_velocity.z = 0
            
            self.imu_msg.linear_acceleration.x = data['accel_y']
            self.imu_msg.linear_acceleration.y = data['accel_x']
            self.imu_msg.linear_acceleration.z = data['accel_z']
            print "oeoeoeo"
            print self.imu_msg.linear_acceleration.x
            print self.imu_msg.linear_acceleration.y
            print self.imu_msg.linear_acceleration.z
            self.imu_msg.header.stamp = rospy.Time.from_sec(data['timestamp'])
            self.imu_data_pub.publish(self.imu_msg)
            
            #br = tf2_ros.transformBroadcaster()
            br = TransformBroadcaster()
            o = [self.imu_msg.orientation.x, self.imu_msg.orientation.y,self.imu_msg.orientation.z,self.imu_msg.orientation.w]
            br.sendTransform((0, 0, 0.5), o, self.imu_msg.header.stamp, '/dummy_imu_link', self.imu_msg.header.frame_id)
            
            r.sleep()
            
    def shutdown(self):
        self.imu_data_pub_shutdown()
        self.imu.close()
        
if __name__ == '__main__':
    try:
        rospy.init_node('imu_node', anonymous = True)
        imu_node = CHR6DNode()
        imu_node.publish_data()
    except rospy.ROSInterruptException:
        imu_node.shutdown()
