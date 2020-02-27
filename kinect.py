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
        # Added fields:
        self.cameraIntrinsicMatrix = np.zeros((3, 3))
        self.cameraDistortionCoeff = np.zeros((5))
        self.depth2rgb_affine3 = np.zeros((3, 3))
        self.cameraFramePoints = np.zeros((5, 3))
        self.camera2world_affine3 = np.zeros((3, 4))
        self.camera2world_affine4 = np.zeros((4, 4))
        self.cameraExtrinsic3 = np.zeros((3, 3))
        self.cameraExtrinsic4 = np.zeros((4, 4))
        self.cameraCalibrated = False
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
        N, K = coord1.shape

        if K == 2:
            A = np.zeros([2 * N, 2 * (K + 1)])
        
            # Build matrix A from pixels
            for i in range(N):
                A[2 * i     , 0 : K] = coord1[i, 0 : K].astype(np.float32) 
                A[2 * i     , K    ] = 1
                A[2 * i + 1 , K + 1 : 2 * K + 1] = coord1[i, 0 : K].astype(np.float32) 
                A[2 * i + 1 , 2 * K + 1   ] = 1
            # print(A)
            # Build b vector
            b = np.zeros([2 * N])
            for i in range(N):
                b[2 * i : 2 * i + 2] = coord2[i, 0 : K].astype(np.float32) 
            # print(b)
            # Compute solution using peseudo inverse
            x = (np.linalg.inv(A.transpose().dot(A))).dot(A.transpose()).dot(b)
            # print(x)     
            transformMatrixTop = np.reshape(x, [2, 3])
            transformMatrixBtm = np.array([0, 0, 1])
            result = np.vstack((transformMatrixTop, transformMatrixBtm))
            # print(result)
            return result
        else:
            A = np.zeros([3 * N, 3 * (K + 1)])
        
            # Build matrix A from pixels
            for i in range(N):
                A[3 * i     , 0 : K] = coord1[i, 0 : K].astype(np.float32) 
                A[3 * i     , K    ] = 1
                A[3 * i + 1 , K + 1 : 2 * K + 1] = coord1[i, 0 : K].astype(np.float32) 
                A[3 * i + 1 , 2 * K + 1   ] = 1
                A[3 * i + 2 , 2 * K + 2 : 3 * K + 2] = coord1[i, 0 : K].astype(np.float32) 
                A[3 * i + 2 , 3 * K + 2   ] = 1
            # Build b vector
            b = np.zeros([3 * N])
            for i in range(N):
                b[3 * i : 3 * i + 3] = coord2[i, 0 : K].astype(np.float32) 
            # print(A)
            # print(b)
            # Compute solution using peseudo inverse
            # print(A.transpose().dot(A))
            x = (np.linalg.inv(A.transpose().dot(A))).dot(A.transpose()).dot(b)
            # print(x)     
            transformMatrixTop = np.reshape(x, [3, 4])
            transformMatrixBtm = np.array([0, 0, 0, 1])
            result = np.vstack((transformMatrixTop, transformMatrixBtm))
            # print(result)
            return result

    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

                    TODO: Using an Affine transformation, transform the depth frame to match the RGB frame using
                    cv2.warpAffine()

        @param      frame  The frame

        @return     { description_of_the_return_value }
        """
        # M = self.getAffineTransform(self.depth_click_points, self.rgb_click_points)
        return cv2.warpAffine(frame,self.depth2rgb_affine,(640, 480))

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

        @param      file  The file
        """
        # fileLines = file.readlines()

        # self.cameraIntrinsicMatrix[0, :] = np.fromstring(fileLines[1])
        # self.cameraIntrinsicMatrix[1, :] = np.fromstring(fileLines[2])
        # self.cameraIntrinsicMatrix[2, :] = np.fromstring(fileLines[3])
        self.cameraIntrinsicMatrix = np.array([[ 524.20336054,0.,300.47947718], [0.,523.18999732,277.66374865], [0.,0.,1.]])
        self.cameraDistortionCoeff = np.array([2.6e-1, -9.14594098e-1, -2.82354497e-3, 1.13680542e-3, 1.20066203e+00])
        # print(self.cameraIntrinsicMatrix)
        # self.cameraDistortionCoeff = np.loadtxt(fileLines[3])


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

    # Added functions:
    def getDepth(self, d):
        z_c = 0.1236 * np.tan(d/2842.5 + 1.1863)
        return z_c

    def pixel2Camera(self, rgbPixel2):
        # Assume pixel has only two value
        rgbPixel3 = np.array([rgbPixel2[1], rgbPixel2[0], 1])
        # depthPixel3 = np.linalg.inv(self.depth2rgb_affine3).dot(rgbPixel3)
        # depthRaw = self.DepthFrameRaw[int(depthPixel3[0]), int(depthPixel3[1])]
        z_c = self.getDepth(self.DepthFrameRaw[rgbPixel2[1], rgbPixel2[0]])
        camerFrameCoord3 = z_c * np.linalg.inv(self.cameraIntrinsicMatrix).dot(rgbPixel3)
        return camerFrameCoord3

    def getWorldCoord(self, rgbPixel2): 
        # Way 1
        # # This is used after calibration is done
        # # Assert rgbPixel only has 2 values
        # # Get depth value from raw depth. Raw depth is already warpped with rgb pixel
        # cameraFrameCoord3 = self.pixel2Camera(rgbPixel2)
        # # print(cameraFrameCoord3)
        # cameraFrameCoord4 = np.array([cameraFrameCoord3[0], cameraFrameCoord3[1], cameraFrameCoord3[2], 1])
        # worldFrameCoord4 = self.camera2world_affine4.dot(cameraFrameCoord4)
        # return worldFrameCoord4[0:3]

        # Way 2
        # z_c = self.getDepth(self.DepthFrameRaw[rgbPixel2[0], rgbPixel2[1]])
        # cameraIntrinsicMatrix3x4 = self.cameraExtrinsic4[0:2, :]
        # A = cameraIntrinsicMatrix3x4
        # A = self.cameraIntrinsicMatrix.dot(self.cameraExtrinsic3)
        # rgbPixel3 = np.array([rgbPixel2[0], rgbPixel2[1], 1])
        # x = np.linalg.inv(A.transpose().dot(A)).dot(A.transpose()).dot(rgbPixel3)
        # print(x)
        # worldCoord3 = x[0:3]
        # # worldCoord3 = np.linalg.inv(A).dot(rgbPixel3)
        # # worldCoord3[0] = -worldCoord3[0]
        # # worldCoord3[1] = -worldCoord3[1]
        # # worldCoord3[2] = 0.93 - z_c
        # return worldCoord3

        # # Way 3
        # z_c = self.getDepth(self.DepthFrameRaw[rgbPixel2[0], rgbPixel2[1]])
        # rgbPixel3 = np.array([rgbPixel2[0], rgbPixel2[1], 1])
        # # cameraFrameCoord3 = z_c * np.linalg.inv(self.cameraIntrinsicMatrix).dot(rgbPixel3)
        # cameraFrameCoord4 = np.array([cameraFrameCoord3[0], cameraFrameCoord3[1], cameraFrameCoord3[2], 1])
        # worldFrameCoord4 = np.linalg.inv(self.cameraExtrinsic4).dot(cameraFrameCoord4)
        # worldCoord3 = worldFrameCoord4[0:3]
        # return cameraFrameCoord3

        # Way 4
        z_c = self.getDepth(self.DepthFrameRaw[rgbPixel2[0], rgbPixel2[1]])
        rgbPixel3 = np.array([rgbPixel2[0], rgbPixel2[1], 1])
        cameraFrameCoord3 = z_c * np.linalg.inv(self.cameraIntrinsicMatrix).dot(rgbPixel3)
        cameraFrameCoord4 = np.array([cameraFrameCoord3[0], cameraFrameCoord3[1], cameraFrameCoord3[2], 1])

        
        # _,extrinsicAffine3,_ = cv2.estimateAffine3D(self.cameraFramePoints, worldCoords)
        worldFrameCoord3 = self.camera2world_affine3.dot(cameraFrameCoord4)
        # cameraFrameCoord4 = np.array([cameraFrameCoord3[0], cameraFrameCoord3[1], cameraFrameCoord3[2], 1])
        # worldFrameCoord4 =o np.linalg.inv(self.cameraExtrinsic4).dot(cameraFrameCoord4)
        # worldCoord3 = worldFrameCoord4[0:3]
        return worldFrameCoord3

    def getExtrinsic(self, worldRefPoints):
        rgb_click_points_float32 = self.rgb_click_points.astype(np.float32)
        _, rot_vec, trans_vec = cv2.solvePnP(worldRefPoints, rgb_click_points_float32, self.cameraIntrinsicMatrix, self.cameraDistortionCoeff, flags = cv2.SOLVEPNP_ITERATIVE)
        # print(trans_vec)
        rot_mat, _ = cv2.Rodrigues(rot_vec) 
        # print(rot_mat)
        self.cameraExtrinsic3 = rot_mat
        self.cameraExtrinsic3[:, 2] = trans_vec[:, 0]
        self.cameraExtrinsic4 = np.column_stack((rot_mat, trans_vec))
        self.cameraExtrinsic4 = np.vstack((self.cameraExtrinsic4, np.array([0,0,0,1])))
        # print(self.cameraExtrinsic4)

        self.camera2world_affine3= self.getAffineTransform(self.cameraFramePoints, worldRefPoints)[0:3, :]


