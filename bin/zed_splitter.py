#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 

class ImageSplitter:
    def __init__(self, subsample_t = 4, subsample_space = 1, queue_size = 1):
        self.subsample_t = subsample_t
        self.count = 0
        self.subsample_space = subsample_space
        self.bridge = CvBridge()
        self.pub_l = rospy.Publisher('/zed/left/image_raw', Image, queue_size = 1)
        self.pub_r = rospy.Publisher('/zed/right/image_raw', Image, queue_size = 1)
        sub = rospy.Subscriber('/zed/image_raw', Image, self.callback, queue_size = queue_size)

    def callback(self, msg):
        try:
            self.count += 1
            #rospy.loginfo(self.count)
            if self.count == self.subsample_t:
                I = self.bridge.imgmsg_to_cv2(msg)
                cols = I.shape[1]
                L = I[:,:cols/2,:]
                R = I[:,cols/2:,:]
                if self.subsample_space > 1:
                    L = L[::self.subsample_space, ::self.subsample_space, :]
                    R = R[::self.subsample_space, ::self.subsample_space, :]
                msg_l = self.bridge.cv2_to_imgmsg(L, 'rgb8')
                msg_l.header = msg.header
                msg_l.header.frame_id = 'zed_left'
                msg_r = self.bridge.cv2_to_imgmsg(R, 'rgb8') 
                msg_r.header = msg.header
                msg_r.header.frame_id = 'zed_right'
                #rospy.loginfo('publish')        
                self.pub_l.publish(msg_l)
                self.pub_r.publish(msg_r)
                self.count = 0
        except  CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('zed_splitter', anonymous=True)
    node = ImageSplitter()
    rospy.spin()


