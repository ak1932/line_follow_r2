#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import controls

class LineFollower(Node):

    def __init__(self):
        super().__init__("line_follower_node")
        
        self.picam2 = Picamera2()
        
        # cmd_vel publisher
        self.speed_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # line_follow_cam publisher
        self.picam2_publisher_raw = self.create_publisher(Image, "/line_follow_cam_raw", 10)
        self.picam2_publisher_black = self.create_publisher(Image, "/line_follow_cam_black", 10)
              
        # Camera Config
        modes = self.picam2.sensor_modes
        mode = modes[1]
        
        # print('mode selected: ', mode)
        
        camera_config = self.picam2.create_still_configuration(raw={'format': mode['unpacked']}, sensor={'output_size': mode['size'], 'bit_depth': mode['bit_depth']}, main={"size":(640, 480)})
        self.picam2.configure(camera_config)
        #self.picam2.set_controls({"AfMode":controls.AfModeEnum.Manual})
        self.picam2.resolution = (640, 360)
        self.picam2.rotation = 180
        
        self.get_logger().info("Cam Initialized")
        
        self.picam2.start()
        
        self.timer = self.create_timer(1/60, self.camera_callback)
        
    def camera_callback(self):
        image = self.picam2.capture_array()
        image = cv2.GaussianBlur(image,(7,7),0)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        Blackline = cv2.inRange(image,np.array([0,0,200], dtype = np.uint8), np.array([180,55,255], dtype = np.uint8))
        
        error = 0.0
        
        kernel = np.ones((3,3), np.uint8)
        Blackline = cv2.erode(Blackline, kernel, iterations=14)
        # Blackline = cv2.dilate(Blackline, kernel, iterations=9)	
        contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45:
                ang = 90 + ang
                
            if w_min < h_min and ang > 0:	  
                ang = (90-ang)*-1
                
            if w_min > h_min and ang < 0:
                ang = 90 + ang
                
            setpoint = 320
            error = int(x_min - setpoint) 
            ang = int(ang)	 
            box = cv2.boxPoints(blackbox)
            box = np.int0(box) 
            
            cv2.drawContours(image,[box],0,(0,0,255),3)	 
            cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)     
        
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        cam_msg_raw = CvBridge().cv2_to_imgmsg(image)
        cam_msg_black = CvBridge().cv2_to_imgmsg(Blackline)
        
        
        self.picam2_publisher_raw.publish(cam_msg_raw)
        self.picam2_publisher_black.publish(cam_msg_black)
        
        speed_cmd = Twist()
        speed_cmd.linear.y = 0.01

        if error == 0.0:
            speed_cmd.angular.z = 0.0
        else:
            speed_cmd.angular.z = (-1 if ang > 0 else 1) * float(90 - abs(ang)) / float(90) 
        self.speed_pub.publish(speed_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
