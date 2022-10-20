#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image

 
class RosbagImageSubscriber(Node): 
    def __init__(self, args):
        super().__init__('ros2bag_sub_node')		

        self.opt_out_file = '/home/aru/m2s2_ws/20221013/1' 	#folder for saving frames TODO: add this as a parameter 
        self.opt_topic = '/image_raw'	               	#topic to subscribe to TODO: add this as a parameter
        self.opt_msg_type = Image                             #message type TODO: add this as a parameter 
        self.bridge = CvBridge()
        self.frame_count = 0

        #create subscription 
        self.subscriber = self.create_subscription(self.opt_msg_type, self.opt_topic, self.callback_image_writer, 0) #args: msg type, topic name, callback, qSize


  
    def callback_image_writer(self, msg):
        '''function that writes frames as cv images in jpeg format with timestamps as the name '''
        frame_no = int(msg.header.frame_id)
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        secs = msg.header.stamp.sec
        nsecs = msg.header.stamp.nanosec
        timestr = str(secs) + "_" + str(nsecs)
        image_name = str(self.opt_out_file)+"/"+ str(frame_no) + "_" + str(timestr)+".jpeg"
        cv2.imwrite(image_name, cv_image)
        self.get_logger().info('Image Received [%i]' %(self.frame_count))
        self.frame_count += 1                    
      
        
        

def main(args=None):
    rclpy.init(args=args)
    node = RosbagImageSubscriber(args) 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
