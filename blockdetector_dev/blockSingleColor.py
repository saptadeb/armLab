import cv2
import numpy as np
import freenect
import time
#img = cv2.imread('ex0_bgr.png',cv2.IMREAD_UNCHANGED)
#cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
#cv2.imshow('window', img)
#cv2.waitKey()

### Kinect.py:
def blockDetector(hsvImg):
	"""!
	@brief      Detect blocks from rgb

				TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
				locations in self.block_detections
	"""
	# Load ref contour
	contour_ref = np.load("contour_ref.npy")

	# Crop the image to ignore backgroud
	borderPoints = np.array([[880, 260], [190, 260],[183, 951],[875, 961]])
	m, n, _ = hsvImg.shape
	ctp1 = borderPoints[0, 0]
	ctp2 = borderPoints[0, 1]
	ctp3 = borderPoints[1, 0]
	ctp4 = borderPoints[1, 1]
	ctp5 = borderPoints[2, 0]
	ctp6 = borderPoints[2, 1]

	hsvImg[:, 0 : ctp2] = np.array([0, 0, 100])
	hsvImg[:, ctp6 : n] = np.array([0, 0, 100])
	hsvImg[0 : ctp3, ctp4 : ctp6] = np.array([0, 0, 100])
	hsvImg[ctp1 : m, ctp2 : ctp6] = np.array([0, 0, 100])
	whiteBoard = np.zeros([m, n, 3], dtype=np.uint8)
	whiteBoard[:, :] = np.array([0, 0, 100], dtype=np.uint8)

	# cv2.namedWindow("cut_window",cv2.WINDOW_AUTOSIZE)
	# cv2.imshow('cut_window', hsvImg)
	# cv2.waitKey()

	# Define color constants
	yellow_lo = np.array([130, 60, 40])
	yellow_hi = np.array([160, 200, 120])
	red2_lo = np.array([160, 140, 80])
	red2_hi = np.array([180, 255, 120])

	kernel = np.ones((5,5),np.uint8)

	inRangeMask = cv2.inRange(hsvImg, yellow_lo, yellow_hi)
	inRangeMask = cv2.morphologyEx(inRangeMask, cv2.MORPH_CLOSE, kernel)
	inRangeMask = cv2.morphologyEx(inRangeMask, cv2.MORPH_OPEN, kernel)
	hsvImg_singleColor = cv2.bitwise_and(hsvImg, hsvImg, mask=inRangeMask)

	k = 43
	if(k == 4):
		inRangeMask2 = cv2.inRange(hsvImg, red2_lo, red2_hi)
		inRangeMask2 = cv2.morphologyEx(inRangeMask2, cv2.MORPH_CLOSE, kernel)
		inRangeMask2 = cv2.morphologyEx(inRangeMask2, cv2.MORPH_OPEN, kernel)
		hsvImg_singleColor2 = cv2.bitwise_and(hsvImg, hsvImg, mask=inRangeMask2)
		hsvImg_singleColor = cv2.bitwise_or(hsvImg_singleColor, hsvImg_singleColor2)
		inRangeMask = cv2.bitwise_or(inRangeMask, inRangeMask2)

	contours, hierarchy = cv2.findContours(inRangeMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	print(len(contours))
	for i in range(len(contours)):
		contour = contours[i]
		rect = cv2.minAreaRect(contour)
		area = cv2.contourArea(contour)
		print(area)
		if (area < 1400 or area > 2600):  # Filter too small ones
			continue
		print(cv2.matchShapes(contour, contour_ref, 1, 0.0))
		if (cv2.matchShapes(contour, contour_ref, 1, 0.0) > 0.3):
			continue
		(center_y, center_x) = rect[0]
		(width, height) = rect[1]
		coutour_orientation = rect[2]
		print("Center X: %f, CenterY : %f, Orientation: %f, Area: %f" %(center_x, center_y, coutour_orientation, area))


	cv2.drawContours(hsvImg_singleColor, contours, -1, (0,255,0), 1)
	cv2.namedWindow("res_window",cv2.WINDOW_AUTOSIZE)
	cv2.imshow('res_window', hsvImg_singleColor)
	cv2.waitKey()

# Provided callback code
font = cv2.FONT_HERSHEY_SIMPLEX
def mouse_callback(event,x,y,flags,param):
	r = img[y][x][2]
	g = img[y][x][1]
	b = img[y][x][0]
	h = hsv[y][x][0]
	s = hsv[y][x][1]
	v = hsv[y][x][2]
	output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
	output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
	output_xy = "X:%d, Y:%d" % (y, x)
	tmp = hsv.copy()
	cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
	cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
	cv2.putText(tmp,output_xy, (10,60), font, 0.5, (0,0,0))
	cv2.imshow('window', tmp)
	if event == cv2.EVENT_LBUTTONDOWN:
		print("bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" %(b,g,r,h,s,v))

### Program Entrance:
img = cv2.imread('testImage.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.namedWindow("window",1)
cv2.imshow('window', hsv)
cv2.setMouseCallback("window",mouse_callback)

blockDetector(hsv)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 27:
        break
cv2.destroyAllWindows()