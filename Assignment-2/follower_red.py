#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
   self.bridge = cv_bridge.CvBridge()
   #cv2.namedWindow("window", 1)
   self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
   self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
   self.twist = Twist()

  def image_callback(self, msg):
   image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
   hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   lower_yellow = numpy.array([ 10, 10, 10])
   upper_yellow = numpy.array([255, 255, 250])
   mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)

   lower_red = numpy.array([0, 50, 50])
   upper_red = numpy.array([10, 255,255])
   mask2 = cv2.inRange(hsv, lower_red, upper_red)

   h, w, d = image.shape
   search_top = 3*h/4
   search_bot = 3*h/4 + 20
   mask1[0:search_top, 0:w] = 0
   mask1[search_bot:h, 0:w] = 0
   M1 = cv2.moments(mask1)
   mask2[0:search_top, 0:w] = 0
   mask2[search_bot:h, 0:w] = 0
   M2 = cv2.moments(mask2)
   if M2['m00']>0:
      cx = int(M2['m10']/M2['m00'])
      cy = int(M2['m01']/M2['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.5
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
   elif M1['m00']>0:
      cx = int(M1['m10']/M1['m00'])
      cy = int(M1['m01']/M1['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.5
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)


   cv2.imshow("window", image)
   cv2.waitKey(3)
 

rospy.init_node('follower')
follower = Follower()
rospy.spin()
