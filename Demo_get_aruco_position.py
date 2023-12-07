import PyCapture2
import cv2
import numpy as np

# Initialize bus and camera
bus = PyCapture2.BusManager()
camera = PyCapture2.Camera()

# Select first camera on the bus
camera.connect(bus.getCameraFromIndex(0))

# Start capture
camera.startCapture()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters_create()
'''
cam_matrix = np.array([[  1.97567771e+04,   0.00000000e+00,   7.45060126e+02],
 [  0.00000000e+00,   1.98777762e+04,   5.21034300e+02],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000]]
)

dist_matrix = np.array([-5.20755217e+00,   2.09653313e+02,  -7.88726235e-03,  -1.37607570e-01,
   -3.27764473e+00])

'''

cam_matrix = np.array([[  2.22780761e+03,   0.00000000e+00 ,  6.79803570e+02],
 [  0.00000000e+00,   2.23516942e+03  , 4.69847972e+02],
 [  0.00000000e+00   ,0.00000000e+00  , 1.00000000e+00]]
)

dist_matrix = np.array([-0.07772019 , 0.27231058 , 0.00259939 , 0.00127269 ,-0.94383607])

i = 0
while True:
	try:
		# Retrieve image from camara in PyCapture2.Image format
		image = camera.retrieveBuffer()

		# Convert from MONO8 to RGB8
		image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

		# Convert image to Numpy array
		rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
		gray_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(gray_cv_image, 80, 255, 0)

		corners, ids, rejected_im_points = cv2.aruco.detectMarkers(thresh, aruco_dict,parameters = params)
		rgb_cv_image = cv2.aruco.drawDetectedMarkers(rgb_cv_image, corners = corners, ids=ids, borderColor=(0, 255, 255))
		r_vec, t_vec= cv2.aruco.estimatePoseSingleMarkers(corners, 0.039, cam_matrix, dist_matrix)

		cv2.aruco.drawAxis(rgb_cv_image, cam_matrix, dist_matrix, r_vec, t_vec, 0.039)
		print("rvec = ", np.rad2deg(r_vec))
		print("tvec = ", 1000 * t_vec)
		print("______")
		# Convert RGB image to BGR image to be shown by OpenCV
		bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)

		# Show image
		cv2.imshow('frame',bgr_cv_image)

		# Wait for key press, stop if the key is q
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.imwrite(f"images/{i}.jpg", bgr_cv_image) 
			i += 1
	except:
		print("Fail")
		continue
