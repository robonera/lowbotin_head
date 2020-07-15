#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError   
import cv2


class BboxDraw:
    def __init__(self, bbox_size  = 4, queue_size = 1):
        self.bbox_s = bbox_size
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('bbox/image_raw', Image, queue_size = 1)
        img_sub = Subscriber('/zed/image_raw', Image)
        detect_sub = Subscriber('/detectnet/detections', Detection2DArray)
        ts = ApproximateTimeSynchronizer([img_sub, detect_sub], queue_size = 10, slop = 5, allow_headerless=True )
        ts.registerCallback(self.callback)
    def callback(self, img, detections):
        rospy.loginfo('one detection to draw')
        try:
            I = self.bridge.imgmsg_to_cv2(img)
            for D in detections.detections:
                pt1 = (D.bbox.center.x - D.bbox.size.x/2,D.bbox.center.y - D.bbox.size.y/2 )
                pt2 = (D.bbox.center.x + D.bbox.size.x/2,D.bbox.center.y + D.bbox.size.y/2 )
                img = cv2.rectangle(img, pt1, pt2, color = (255, 0, 0), thickness = self.bbox_s)
            self.pub.publish(self.bridge.cv2_to_imgmsg(img,'rgb8'))
        except  CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('bbox_drawer', anonymous=True)
    node = BboxDraw()
    rospy.spin()




