from imutils.video import WebcamVideoStream
import argparse
import imutils
import cv2
import serial
import numpy as np
import cv2
import time
from imutils.object_detection import non_max_suppression
import sys
import serial
import time

SERIAL = False
xres = 640
yres = 480


if SERIAL:	
	ser = serial.Serial()
	ser.baudrate = 115200
	# ser.port = '/dev/ttyUSB2'
	ser.port = 'COM6'
	ser.timeout = 5
	ser.open()

vs = WebcamVideoStream(src=0).start()
file_path = "haarcascade_frontalface_default.xml"
cascade = cv2.CascadeClassifier(file_path)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
time.sleep(2.0)

try:
	while True:
		t1 = time.time()

		# Capture frame-by-frame
		frame = vs.read();

		# Convert the image from OpenCV BGR format to matplotlib RGB format
		# to display the image
		# frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		faces = cascade.detectMultiScale(
		gray,
		scaleFactor=1.2,
		minNeighbors=7,
		minSize=(30, 30),
		flags=cv2.CASCADE_SCALE_IMAGE
		)

		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
			cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# Calculate message
		if len(faces)!=0 and SERIAL:
			# Look at the closest face
			#diff = abs((faces[:,0]+faces[:,2]/2)-xres/2)
			#f = np.argmin(diff)
			f = np.argmax(faces[:,2])
			param1 = xres-int((faces[f][0]+faces[f][2]/2))
			param2 = int((faces[f][1]+faces[f][3]/2))
			if param2 < 50 or param2 > 400:
				param2 = yres/2
			MESSAGE = [str(param1).zfill(3),str(param2).zfill(3),'\n']
			MESSAGE = ''.join(MESSAGE)
			ser.write(MESSAGE.encode("utf-8"))
			# time.sleep(0.01)

		t2 = time.time()

		if t2 != t1:
			print("%f FPS" % (1/(t2-t1)))

except KeyboardInterrupt:

	if SERIAL:
		ser.close();
	# do a bit of cleanup
	cv2.destroyAllWindows()
	vs.stop()
	exit()
