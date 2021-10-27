import cv2
import numpy as np
import glob
import time
dirpath = "/home/guru/catkin_ws/src/g_bot/g_turtle/src/camera_calib/images"
dirpath_save = "/home/guru/catkin_ws/src/g_bot/g_turtle/src/camera_calib/"
prefix = "img"
image_format = "jpg"
square_size = 2.1
width = 9
height = 7

#defining the variables

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((height*width, 3), np.float32)
objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
objp = objp * square_size
objpoints = []
imgpoints = []
images = glob.glob(dirpath+'/' + prefix + '*.' + image_format)

#finding the corners in the chessboard in each image

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        cv2.imshow("guru",img)
        time.sleep(1)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
#caliberating

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#saving a sample image, after undistorting

h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite(dirpath_save + 'calibresult.png',dst)

#calculating the reprojection error

tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

#printing the error and camera matrix

print ("total error: ", tot_error/len(objpoints))
if ret:
    print(mtx)
    print(dist)

#saving the camera matrix and distortion coefficents
np.savetxt(dirpath_save+'camera_dist.csv', dist, delimiter=',')
np.savetxt(dirpath_save+'camera_mtx.csv', mtx, delimiter=',')