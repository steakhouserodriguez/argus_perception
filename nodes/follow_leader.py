#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
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
#  * Neither the name of the Willow Garage nor the names of its
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
#!/usr/bin/env python

import roslib
roslib.load_manifest('argus_perception')
import rospy
from  geometry_msgs.msg import Twist
import math
import tf
import time
import sys, subprocess, time, tempfile, threading, tty, datetime

def QuitApril():
    april.kill()
    
    
if __name__ == '__main__':
    global april
    rospy.init_node('follow_leader')

    pub = rospy.Publisher('safe_cmd_vel', Twist)
    rospy.on_shutdown(QuitApril)

    print "Starting follow_leader..."

    #listener = tf.TransformListener()
    s = subprocess.Popen(args=['rospack','find', 'argus_perception'], shell=False, stdout=subprocess.PIPE, bufsize=0)
    path = s.stdout.readline()
    path = path[0:len(path)-1]  + '/nodes/AprilTagFinder'
    print "using path=" + path

    april = subprocess.Popen(args = ['sudo', 'nice', '-n', str(-10), path ,'camera://0', 'TAG16H5'], shell = False, stdout = subprocess.PIPE, bufsize = 0)


    #rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():

        line = april.stdout.readline()
        #print "received line=" + line
        split = line.split(',')
        if split[0] == 'pos' and len(split) == 5:
            print "received tags=" + str(split)
            tag0 = split[1:3]
            print "tag0=" + str(tag0)

            msg = Twist()
            msg.linear.x = 0
            error = 320 - float(tag0[0])


            if error > 100:
                msg.linear.x = 0
                msg.angular.z = 1.5
            elif error < -100:
                msg.linear.x = 0
                msg.angular.z = -1.5
            else:
                msg.linear.x = 1.75
                msg.angular.z = 0

            '''
            if  float(tag0[0]) > 320:
                msg.angular.z = -2.0
            else:
                msg.angular.z = 2.0
            '''
            
            #msg.angular.z = (320 - float(tag0[0]))/320.0
            pub.publish(msg)
            
        '''
        try:
        (trans, rot) = listener.lookupTransform('/usb_cam', '/april.tag.Tag16h5.0', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        print "tf rotations=" + str(euler)
        except (tf.LookupException, tf.ConnectivityException):
        continue
        '''

