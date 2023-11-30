# Import required modules 
import cv2 
import numpy as np 
import os 
import glob 


# Define the dimensions of checkerboard 
CHECKERBOARD = (4, 6) 


# stop the iteration when specified 
# accuracy, epsilon, is reached or 
# specified number of iterations are completed. 
criteria = (cv2.TERM_CRITERIA_EPS +
			cv2.TERM_CRITERIA_MAX_ITER, 35, 0.001) 


# Vector for 3D points 
threedpoints = [] 

# Vector for 2D points 
twodpoints = [] 

"""
40x 20 mm 8x12 png
[[  2.22780761e+03   0.00000000e+00   6.79803570e+02]
 [  0.00000000e+00   2.23516942e+03   4.69847972e+02]
 [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]

[[-0.07772019  0.27231058  0.00259939  0.00127269 -0.94383607]]

20x 20 mm 8x12 jpg
[[  1.62590729e+04   0.00000000e+00   6.47834867e+02]
 [  0.00000000e+00   1.62476841e+04   3.16185817e+02]
 [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
dist_matrix = np.array([-5.20755217e+00,   2.09653313e+02,  -7.88726235e-03,  -1.37607570e-01,
   -3.27764473e+00])

20x 10mm 17x25 bmp
[[  1.47970904e+04   0.00000000e+00   8.02708012e+02]
 [  0.00000000e+00   1.48539513e+04   3.83738477e+02]
 [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
[ -4.21304618e+00   4.23254102e+02   1.83874158e-02  -8.35195764e-02
    2.57509132e+00]


"""

# 3D points real world coordinates 
objectp3d = np.zeros((1, CHECKERBOARD[0] 
					* CHECKERBOARD[1], 
					3), np.float32) 
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) 
prev_img_shape = None


# Extracting path of individual image stored 
# in a given directory. Since no path is 
# specified, it will take current directory 
# jpg files alone 
images = glob.glob('./images/*.png') 

for filename in images: 
	image = cv2.imread(filename) 
	grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 

	# Find the chess board corners 
	# If desired number of corners are 
	# found in the image then ret = true 
	ret, corners = cv2.findChessboardCorners( 
					grayColor, CHECKERBOARD, 
					cv2.CALIB_CB_ADAPTIVE_THRESH 
					+ cv2.CALIB_CB_FAST_CHECK +
					cv2.CALIB_CB_NORMALIZE_IMAGE) 
	#print(corners)
	# If desired number of corners can be detected then, 
	# refine the pixel coordinates and display 
	# them on the images of checker board 
	if ret == True: 
		threedpoints.append(objectp3d) 

		# Refining pixel coordinates 
		# for given 2d points. 
		corners2 = cv2.cornerSubPix( 
			grayColor, corners, (11, 11), (-1, -1), criteria) 

		twodpoints.append(corners2) 

		# Draw and display the corners 
		image = cv2.drawChessboardCorners(image, 
										CHECKERBOARD, 
										corners2, ret) 

	#cv2.imshow('img', image) 
	#cv2.waitKey(0) 

cv2.destroyAllWindows() 

h, w = image.shape[:2] 


# Perform camera calibration by 
# passing the value of above found out 3D points (threedpoints) 
# and its corresponding pixel coordinates of the 
# detected corners (twodpoints) 
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera( 
	threedpoints, twodpoints, grayColor.shape[::-1], None, None) 


# Displaying required output 
print(" Camera matrix:") 
print(matrix) 

print("\n Distortion coefficient:") 
print(distortion) 

print("\n Rotation Vectors:") 
print(r_vecs) 

print("\n Translation Vectors:") 
print(t_vecs) 

