import cv2
import numpy as np

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
	# Crop the image to ignore backgroud
	borderPoints = np.array([[442, 130], [94, 130],[97, 476],[442, 476]])
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
	#cv2.namedWindow("cut_window",cv2.WINDOW_AUTOSIZE)
	#cv2.imshow('cut_window', hsvImg)
	#cv2.waitKey()

	# Define color constants
	#yellow_lo = np.array([23 ,200,240])
	#yellow_hi = np.array([35 ,255,255])
	#orange_lo = np.array([9  ,190,190])
	#orange_hi = np.array([14 ,255,210])
	#pink_lo = np.array([170, 180, 190])
	#pink_hi = np.array([175, 200, 210])
	#black_lo = np.array([0, 0, 0])
	#black_hi = np.array([180, 255, 50])
	#red_lo = np.array([170, 190, 130])
	#red_hi = np.array([180, 230, 170])
	#pink_lo = np.array([150, 80, 120])
	#pink_hi = np.array([165, 130, 150])
	#green_lo = np.array([50, 40, 115])
	#green_hi = np.array([70, 75, 135])
	yellow_lo = np.array([150, 80, 120])
	yellow_hi = np.array([165, 130, 150])

	kernel = np.ones((5,5),np.uint8)

	inRangeMask = cv2.inRange(hsvImg, yellow_lo, yellow_hi)
	inRangeMask = cv2.morphologyEx(inRangeMask, cv2.MORPH_CLOSE, kernel)
	inRangeMask = cv2.morphologyEx(inRangeMask, cv2.MORPH_OPEN, kernel)
	hsvImg_singleColor = cv2.bitwise_and(hsvImg,hsvImg,mask = inRangeMask)
	contours, hierarchy = cv2.findContours(inRangeMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#inRangeMask = cv2.inRange(hsvImg, yellow_lo, yellow_hi)
		#hsvImg_singleColor = cv2.bitwise_and(hsvImg,hsvImg,mask = inRangeMask)
		#hsvImg_singleColor = np.where(hsvImg_singleColor[:,:] == [0,0,0], whiteBoard, hsvImg_singleColor)
		#hsvImg_singleColor = cv2.morphologyEx(hsvImg_singleColor, cv2.MORPH_CLOSE, kernel)
		#hsvImg_singleColor = cv2.morphologyEx(hsvImg_singleColor, cv2.MORPH_OPEN, kernel)

		#cv2.namedWindow("res_window",cv2.WINDOW_AUTOSIZE)
		#cv2.imshow('res_window', hsvImg_singleColor)
		#cv2.waitKey()

		#hsvImg_singleColor_rgb = cv2.cvtColor(hsvImg_singleColor,	 cv2.COLOR_HSV2RGB)
		#hsvImg_singleColor_rgbGrey = cv2.cvtColor(hsvImg_singleColor_rgb,cv2.COLOR_RGB2GRAY)
		#cv2.namedWindow("res_window",cv2.WINDOW_AUTOSIZE)
		#cv2.imshow('res_window', hsvImg_singleColor_rgb)
		#cv2.waitKey()
		#ret,thresh = cv2.threshold(hsvImg_singleColor_rgb,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
		#contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	print(len(contours))
	for i in range(len(contours)):
		contour = contours[i]
		rect = cv2.minAreaRect(contour)
		area = cv2.contourArea(contour)
		(center_x, center_y) = rect[0]
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
img = cv2.imread('ex0_bgr.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.namedWindow("window",1)
cv2.imshow('window', img)
cv2.setMouseCallback("window",mouse_callback)

blockDetector(hsv)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 27:
        break
cv2.destroyAllWindows()