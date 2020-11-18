#!/usr/bin/env python3.7
import rospy
import signal
import cv2
import numpy as np
from sensor_msgs.msg import Image
from imgmsg import CvBridge
rospy.init_node('imagepublisher', anonymous=True)
image_pub = rospy.Publisher("rgb/image",Image,queue_size = 1)
vid = cv2.VideoCapture(0) 
vid.set(3,320)
vid.set(4,240)
bridge = CvBridge()
def keyboardInterruptHandler(signal, frame):
    print("closing the camera!!!!!")
    vid.release()
    print("exiting....")
    exit(0)
while True:
    ret, frame = vid.read()
    #frame = np.asarray(frame)
    frame = cv2.resize(frame,(150,100))
    if ret:
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image = bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        image_pub.publish(image)
    signal.signal(signal.SIGINT, keyboardInterruptHandler)
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
vid.release()  
cv2.destroyAllWindows() 
