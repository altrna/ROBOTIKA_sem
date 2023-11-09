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

i = 0
while True:
  # Retrieve image from camara in PyCapture2.Image format
  image = camera.retrieveBuffer()

  # Convert from MONO8 to RGB8
  image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

  # Convert image to Numpy array
  rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))

  corners, ids, rejected_im_points = cv2.aruco.detectMarkers(rgb_cv_image, aruco_dict,parameters = params)
  rgb_cv_image = cv2.aruco.drawDetectedMarkers(rgb_cv_image, corners = corners, ids=ids, borderColor=(0, 255, 255))
  print(corners, "\n")

  # Convert RGB image to BGR image to be shown by OpenCV
  bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)

  # Show image
  cv2.imshow('frame',bgr_cv_image)

  # Wait for key press, stop if the key is q
  if cv2.waitKey(1) & 0xFF == ord('q'):
      cv2.imwrite(f"images/{i}.bmp", bgr_cv_image) 
      i += 1
