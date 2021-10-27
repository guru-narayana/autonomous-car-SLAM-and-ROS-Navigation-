#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
from geometry_msgs.msg import Pose
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
bridge = CvBridge()
rospy.init_node('camera_client')
pub = rospy.Publisher("object_pose",Pose,queue_size=10)
distortion_coefficients = np.genfromtxt('/home/guru/catkin_ws/src/g_bot/arm_control/src/camera_dist.csv', delimiter=',')
matrix_coefficients = np.genfromtxt('/home/guru/catkin_ws/src/g_bot/arm_control/src/camera_mtx.csv', delimiter=',')
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

# Create grid board object we're using in our stream
#board = aruco.GridBoard_create(
#        markersX=2,
#        markersY=2,
#        markerLength=1.5,
#        markerSeparation=0.5,
#        dictionary=ARUCO_DICT)
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)
ids_id = np.array([[0],[1],[2],[3],[4],[5]], dtype=np.int32)
d = 3
s = 0.5
objPoints = np.array([
     np.array([[-d/2,d/2,(d/2)+s],[d/2,d/2,(d/2)+s],[d/2,-d/2,(d/2)+s],[-d/2,-d/2,(d/2)+s]],  dtype=np.float32),
     np.array([[(d/2)+s,d/2,d/2],[(d/2)+s,d/2,-d/2],[(d/2)+s,-d/2,-d/2],[(d/2)+s,-d/2,d/2]], dtype=np.float32),
     np.array([[d/2,d/2,-((d/2)+s)],[-d/2,d/2,-((d/2)+s)],[-d/2,-d/2,-((d/2)+s)],[d/2,-d/2,-((d/2)+s)]], dtype=np.float32),
     np.array([[-((d/2)+s),d/2,-d/2],[-((d/2)+s),d/2,d/2],[-((d/2)+s),-d/2,d/2],[-((d/2)+s),-d/2,-d/2]],  dtype=np.float32),
     np.array([[-d/2,(d/2)+s,-d/2],[d/2,(d/2)+s,-d/2],[d/2,(d/2)+s,d/2],[-d/2,(d/2)+s,d/2]], dtype=np.float32),
     np.array([[-d/2,-((d/2)+s),d/2],[d/2,-((d/2)+s),d/2],[d/2,-((d/2)+s),-d/2],[-d/2,-((d/2)+s),-d/2]], dtype=np.float32)
    ])
print(objPoints)
board = aruco.Board_create(objPoints, ARUCO_DICT, ids_id)

def inversePerspective(rvec, tvec):
    R,_ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec

def img_callback(data):
    pos = Pose()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.addWeighted(gray,1.2,0,0,0)
    #corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
    #                                                            parameters=parameters,
    #                                                            cameraMatrix=matrix_coefficients,
    #                                                            distCoeff=distortion_coefficients)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board
    corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                image = gray,
                board = board,
                detectedCorners = corners,
                detectedIds = ids,
                rejectedCorners = rejectedImgPoints,
                cameraMatrix = matrix_coefficients,
                distCoeffs = distortion_coefficients)
    if ids is not None and len(ids) > 0:
        
        frame = aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
        # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
        pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, matrix_coefficients, distortion_coefficients,rvec=None,tvec=None)
        if pose:
            # Draw the camera posture calculated from the gridboard
            frame = aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 10)
            #rvec,tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))
            #rvec,tvec = inversePerspective(rvec,tvec)
            print(tvec)
            pos.position.x = tvec[0]
            pos.position.y = tvec[1]
            pos.position.z = tvec[2]
            pub.publish(pos)
    cv2.imshow("image",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        pass
    
rospy.Subscriber("rgb/image",Image,img_callback)
rospy.spin()