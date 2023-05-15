#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import cv2, cv_bridge, numpy

class Bowling:
    def __init__(self):
        rospy.init_node('bowling')
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.fiducial_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray,  self.fiducial_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.phase = 0
                
        self.is_targeted = False
        self.last_x = 0.0
        self.min_front = 10000
        self.bridge = cv_bridge.CvBridge()
        self.image = None

        self.points = {}
        self.fid_msg = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.hash = set()
        
        self.pincer_state = "release"
        
        self.run()
        
    def scan_callback(self, msg):
        range_front = []
        range_front[:5] = msg.ranges[5:0:-1]  
        range_front[5:] = msg.ranges[-1:-5:-1]

        self.min_front = range_front[0]
        for range in range_front:
            self.min_front = min(range, self.min_front)
            
        print("min_front: ", self.min_front)


  
  
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')


    def fiducial_callback(self, msg):
        self.fid_msg = msg
        if len(msg.transforms) > 0:
            for i in range(len(msg.transforms)):

                if msg.transforms[i].fiducial_id == 0 and self.phase == 0:
                    self.phase = 1  # fid_0 detected, drive to it

                p = Point()
                delta_x = msg.transforms[0].transform.translation.z
                delta_y = -msg.transforms[0].transform.translation.x
            

            
                p.x = self.x + delta_x*math.cos(self.theta)-delta_y*math.sin(self.theta)
                p.y = self.y + delta_x*math.sin(self.theta)+delta_y*math.cos(self.theta)
                p.z = 0.0
                self.points[msg.transforms[i].fiducial_id] = p

                #print(p)
                
            #self.is_detected = True
            print(self.points)
            #print(msg.transforms)


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
 

    def rotate(self, speed):
        print("now rotate")
        twist = Twist()
        twist.angular.z = speed
        self.twist_pub.publish(twist)
        
    def move(self, speed):
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = speed
        self.twist_pub.publish(twist)
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    
    # go to a fiducial marker
    def goto_pnt(self, n):
        
        goal = self.points[n]
        print("goal: ", goal)
        print("current location: ", "(", self.x, ",", self.y, ")")
        twist = Twist()
        inc_x = goal.x - self.x
        inc_y = goal.y - self.y
        
        angle_to_goal = math.atan2(inc_y, inc_x)
        print("angle to goal:", angle_to_goal)
        dis_to_goal = math.sqrt(inc_x**2 + inc_y**2)
        
        if dis_to_goal < 0.05:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.phase == 1:
                self.phase = 2
        elif angle_to_goal - self.theta > 0.1:
            twist.linear.x = 0.0
            twist.angular.z = 0.1
        elif angle_to_goal - self.theta < -0.1:
            twist.linear.x = 0.0
            twist.angular.z = -0.1
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.0


        
        self.twist_pub.publish(twist)
    
    # align to the pins(fiducial marker 1)
    def align(self):

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 20,  100,  100])
        upper_yellow = numpy.array([30, 255, 255])
        h, w, d = self.image.shape
    
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        M = cv2.moments(mask)
        print("M00: ", M['m00'])
        if self.phase == 2:
            if M['m00'] != 0:

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                delta_x = cx - 0.5*w
                if abs(delta_x) < 0.05 * w:
                    self.stop()
                    self.phase = 3
                elif delta_x > 0:
                    self.rotate(-0.1)
                else:
                    self.rotate(0.1)
            else:
                self.rotate(1.0)
                
 

        
    def run(self):
        r = rospy.Rate(10)
        r.sleep()
        while not rospy.is_shutdown():
            print("now in phase ", self.phase)

            if self.phase == 0:
                self.rotate(0.3)  # try to find fid_0
            elif self.phase == 1:
                self.goto_pnt(0)  # fid_0 found, go to fid_0
            elif self.phase == 2:
                rospy.sleep(1)
                self.align()
            elif self.phase == 3:
                rospy.sleep(2)

                for i in range(90):
                    self.move(0.2)
                    r.sleep()
                self.stop()
                self.phase = 4



            r.sleep()

if __name__ == '__main__':
    bowling = Bowling()
    rospy.spin()



