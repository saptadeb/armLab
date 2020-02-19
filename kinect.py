"""!
Class to represent the kinect.
"""

import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import os
script_path = os.path.dirname(os.path.realpath(__file__))

class Kinect():
    """!
    @brief      This class describes a kinect.
    """

    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.VideoFrame = np.array([])
        self.DepthFrameRaw = np.array([]).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRGB=np.array([])

        """initialize kinect & turn off auto gain and whitebalance"""
        freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)
        # print(freenect.sync_set_autoexposure(False))
        freenect.sync_set_autoexposure(False)
        # print(freenect.sync_set_whitebalance(False))
        freenect.sync_set_whitebalance(False)
        """check depth returns a frame, and flag kinectConnected"""
        if(freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT) == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True

        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)

        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])

    def toggleExposure(self, state):
        """!
        @brief      Toggle auto exposure

        @param      state  False turns off auto exposure True turns it on
        """
        if state == False:
            freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)
            # print(freenect.sync_set_autoexposure(False))
            freenect.sync_set_autoexposure(False)
            # print(freenect.sync_set_whitebalance(False))
            freenect.sync_set_whitebalance(False)
        else:
            freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)
            # print(freenect.sync_set_autoexposure(True))
            freenect.sync_set_autoexposure(True)
            # print(freenect.sync_set_whitebalance(True))
            freenect.sync_set_whitebalance(True)

    def captureVideoFrame(self):
        """!
        @brief Capture frame from Kinect, format is 24bit RGB
        """
        if(self.kinectConnected):
           self.VideoFrame = freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame,self.block_contours,-1,(255,0,255),3)

    def captureDepthFrame(self):
        """!
        @brief Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.DepthFrameRaw = self.registerDepthFrame(freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT)[0])
            else:
                self.DepthFrameRaw = freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT)[0]
        else:
            self.loadDepthFrame()

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[...,0] = self.DepthFrameRaw
        self.DepthFrameHSV[...,1] = 0x9F
        self.DepthFrameHSV[...,2] = 0xFF
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread(script_path + "/data/rgb_image.png",cv2.IMREAD_UNCHANGED),cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread(script_path + "/data/raw_depth.png",0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertQtDepthFrame(self):
       """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
       try:
           img = QImage(self.DepthFrameRGB,
                            self.DepthFrameRGB.shape[1],
                            self.DepthFrameRGB.shape[0],
                            QImage.Format_RGB888
                            )
           return img
       except:
           return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

                    TODO: Rewrite this function to take in an arbitrary number of coordinates and find the transform without
                    using cv2 functions

        @param      coord1  The coordinate 1
        @param      coord2  The coordinate 2

        @return     Affine transform between coordinates.
        """
        # pts1 = coord1[0:3].astype(np.float32)
        # pts2 = coord2[0:3].astype(np.float32)
        # print(cv2.getAffineTransform(pts1, pts2))
        # return cv2.getAffineTransform(pts1, pts2)

        # Assume coord1 is an array of source/pixel and coord2 is known dst coordinates
        coord1 = np.hstack((coord1, np.ones([coord1.shape[0],1])))
        N, K = coord1.shape
        M, L = coord2.shape
        A = np.zeros([2 * N, 2 * K])

        # Build matrix A from pixels
        for i in range(N):
            A[2 * i     , 0 : K] = coord1[i, 0 : K].astype(np.float32)
            #A[2 * i     , K    ] = 1
            A[2 * i + 1 , K : ] = coord1[i, 0 : K].astype(np.float32)
            #A[2 * i + 1 , K    ] = 1

        # Build b vector
        b = np.zeros([2 * N])
        for i in range(N):
            b[2 * i : 2 * i + 2] = coord2[i, 0 : L].astype(np.float32)

        # Compute solution using peseudo inverse
        x = (np.linalg.inv(A.transpose().dot(A))).dot(A.transpose()).dot(b)
        transformMatrixTop = np.reshape(x, [2, 3])
        # transformMatrixBtm = np.array([0, 0, 1])
        # result = np.concatenate((transformMatrixTop, transformMatrixBtm), axis=0)
        result = transformMatrixTop
        return result

    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

                    TODO: Using an Affine transformation, transform the depth frame to match the RGB frame using
                    cv2.warpAffine()

        @param      frame  The frame

        @return     { description_of_the_return_value }
        """
        M = self.getAffineTransform(self.rgb_click_points, self.depth_click_points)
        return cv2.warpAffine(frame,M,frame.shape)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        pass

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        pass
