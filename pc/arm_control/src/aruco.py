import numpy
import cv2
import cv2.aruco as aruco

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)
ids = numpy.array([[0],[1],[2],[3]])
d = 5
s = 1
objPoints = np.array([
    [[0,0,0],[d,0,0],[d,d,0],[0,d,0]],
    [[d+s,0,s],[d+s,0,d+s],[d+s,d,s],[d+s,d,d+s]],
    [[d,0,d+2*s],[0,0,d+2*s],[0,d,d+2*s],[d,d,d+2*s]],
    [[-s,0,d+s],[-s,0,s],[-s,d,s],[-s,d,d+s]],
    [[0,-s,d+s],[d,-s,d+s],[d,-s,s],[0,-s,s]]
    ])
board = aruco.Board_create(objPoints.astype(numpy.float32), ARUCO_DICT, ids)