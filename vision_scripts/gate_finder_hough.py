#!/usr/bin/env python
import rospy
from rospkg import RosPack
import signal
import sys
import cv2
import traceback
import numpy as np

from navigation.color_classifiers import SingleGaussian, GMM, MeanThreshold, straight_threshold
from navigation.gate_finder import gateFinder
from navigation.msg import WP_Msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from geometry import *



class gateFinder():
    """
    Class to find the position and orientation of the gate given a masked image.
    Gate position is in meters; axis is defined as +x in front of, +y to the
    left of, and +z above UAV. Orientation is in radians CCW, where zero radians
    is looking directly at the open end of the gate.
    """
    
    def __init__(self,
                 x_res,
                 y_res,
                 erosion_kernel=np.ones((3,3), dtype=np.uint8),
                 erosion_numIterations=1,
                 dilation_kernel=np.ones((3,3), dtype=np.uint8),
                 dilation_numIterations=2,
                 hough_pixelRes=1,
                 hough_angleRes=np.pi/180.,
                 hough_minNumIntersections=50,
                 hough_minLineLength=50,
                 hough_maxLineGap=100,
                 hough_lineExtension=25.0,
                 canny_minVal=50,
                 canny_maxVal=250,
                 canny_grad=3,
                 blob_minSize=10000,
                 record_image=True):

        # define horizontal and vertical resolution
        self.x_res = x_res
        self.y_res = y_res
        self.record_image = record_image

        # define src (unprocessed masked image) and color and binary dst (processed mask image)
        self.src = np.zeros((self.y_res, self.x_res), dtype=np.uint8)
        self.dst = np.zeros((self.y_res, self.x_res), dtype=np.uint8)
        if self.record_image:
            self.dst_color = np.zeros((self.y_res, self.x_res, 3), dtype=np.uint8)

        # define kernel and num iterations for erosion and dilation
        self._erosion_kernel = erosion_kernel
        self._dilation_kernel = dilation_kernel
        self._erosion_numIterations = erosion_numIterations
        self._dilation_numIterations = dilation_numIterations

        # define hough threshold values and property to hold line
        # data; line data is in form [x0, y0, x1, y1]
        self._hough_pixelRes = hough_pixelRes
        self._hough_angleRes = hough_angleRes
        self._hough_minNumIntersections = hough_minNumIntersections
        self._hough_minLineLength = hough_minLineLength
        self._hough_maxLineGap = hough_maxLineGap
        self._hough_lines = np.zeros((1, 1, 4), dtype=np.int32)
        self._hough_lineExtension = hough_lineExtension
        self._hough_intersectionPoints = []

        # define Canny edge detection threshold and gradient values
        self._canny_minVal = canny_minVal
        self._canny_maxVal = canny_maxVal
        self._canny_grad = canny_grad

        # define minimum size for blob removal
        self._blob_minSize = blob_minSize

        # define gate center and corner locations in the image
        self._gate_centerPixel_x = 0
        self._gate_centerPixel_y = 0
        self._gate_corners = np.array([], dtype=np.float64)

        # define gate position and orientation
        self.gate_pos_x = 0.0
        self.gate_pos_y = 0.0
        self.gate_pos_z = 0.0
        self.gate_rot = 0.0

    def updateSrc(self, src):
        """
        Empties current data and updates source image data.
        """
        self._clearData()
        self.src = src
        self.dst = src

    def processSrc(self):
        """
        Runs the src data through the custom totally not buggy process.
        """

        # remove small blobs
        #self._removeAreas()
    
        # erode and dilate 
        self._erode()
        self._dilate()
    
        # remove small blobs
        self._removeAreas()
    
        self._dilate()
        # get edges and lines
        self._getCannyEdges()
        self._getHoughLines()
    
        # extend hough lines
        self._extendHoughLines()
    
        # create color dst image
        self._createColorDst()
    
        # draw hough lines
        self._drawHoughLines()

        # get hough intersection points
        if not self._getHoughIntersections():
            return

        # get gate corners
        self._findGateCorners()

        # get gate center pixel data
        # self._findGateCenter()

    def _clearData(self):
        """
        Clears all public properties to default values.
        """
        self.src = np.zeros((self.y_res, self.x_res), dtype=np.uint8)
        self.dst = np.zeros((self.y_res, self.x_res), dtype=np.uint8)
        if self.record_image:
            self.dst_color = np.zeros((self.y_res, self.x_res, 3), dtype=np.uint8)
        self.gate_pos_x = 0.0
        self.gate_pos_y = 0.0
        self.gate_pos_z = 0.0
        self.gate_rot = 0.0

    def _createColorDst(self):
        """
        Creates a copy of the current binary destination image to a color copy.
        """
        if self.record_image:
            self.dst_color = cv.cvtColor(self.dst, cv.COLOR_GRAY2BGR)

    def _erode(self, kernel=None, numIterations=None):
        """
        Erodes destination image using a sepcified kernel for a set number of iterations
        """

        erosion_kernel = self._erosion_kernel if kernel == None else kernel
        erosion_numIterations = self._erosion_numIterations if numIterations == None else numIterations

        self.dst = cv.erode(self.dst, erosion_kernel, iterations=erosion_numIterations) if erosion_numIterations else self.dst

    def _dilate(self, kernel=None, numIterations=None):
        """
        Dilates destination image using a sepcified kernel for a set number of iterations
        """

        dilation_kernel = self._dilation_kernel if kernel == None else kernel
        dilation_numIterations = self._dilation_numIterations if numIterations == None else numIterations

        self.dst = cv.dilate(self.dst, dilation_kernel, iterations=dilation_numIterations) if dilation_numIterations else self.dst

    def _getCannyEdges(self, minVal=None, maxVal=None, grad=None):
        """
        Updates destination image to extract edges via Canny edge detection.
        """
        canny_minVal = self._canny_minVal if minVal == None else minVal
        canny_maxVal = self._canny_maxVal if maxVal == None else maxVal
        canny_grad   = self._canny_grad   if grad   == None else grad

        self.dst = cv.Canny(self.dst, canny_minVal, canny_maxVal, None, canny_grad)

    def _getHoughLines(self, pixelRes=None, angleRes=None, minNumIntersections=None, minLineLength=None, maxLineGap=None):
        """
        Determines the edges of a given source image via Canny transform for edge detection
        and a probablistic Hough transform for line detection. Outputs the set of edges and
        lines.

        """

        hough_pixelRes = self._hough_pixelRes if pixelRes == None else pixelRes
        hough_angleRes = self._hough_angleRes if angleRes == None else angleRes
        hough_minNumIntersections = self._hough_minNumIntersections if minNumIntersections == None else minNumIntersections
        hough_minLineLength       = self._hough_minLineLength       if minLineLength       == None else minLineLength
        hough_maxLineGap          = self._hough_maxLineGap          if maxLineGap          == None else maxLineGap
       
        # get lines from probabilistic hough xform
        lines = cv.HoughLinesP(self.dst, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)
        try:
            self._hough_lines = [Line(Point(line[0,0],line[0,1]), Point(line[0,2],line[0,3])) for line in lines]
        except TypeError as e:
            rospy.logger("Failed to detect ANY hough lines")
            self._hough_lines = []


    def _extendHoughLines(self, lineExtension=None):
        """
        Extends current set of hough lines by some a set amount.
        """ 

        lineExtension = self._hough_lineExtension if lineExtension == None else lineExtension

        self._hough_lines = [line.extend(lineExtension) for line in self._hough_lines]

    def _drawHoughLines(self):
        """
        Draws current hough lines on color destination image.
        """

        # check that lines exist
        if self._hough_lines == None:
            print "ERROR: no hough lines for drawHoughLines."
            return None

        # draw the lines
        if self.record_image:
            for line in self._hough_lines:
                cv.line(self.dst_color, line.point_0.returnTuple(), line.point_1.returnTuple(), (0,0,255), 1, cv.LINE_AA)

    def _removeAreas(self, minSize=None):
        """
        Removes areas from the destination image that are below a specified minimum size.
        """

        minSize = self._blob_minSize if minSize == None else minSize

        # get connectivity data
        nb_components, output, stats, centroids = cv.connectedComponentsWithStats(self.dst, connectivity=8)
    
        # remove background image
        stats = stats[1:]; nb_components -= 1
        
        # create return image
        self.dst = np.zeros((output.shape), dtype=np.uint8)

        # for every component in the image, keep only if it's above minSize
        for i in range(nb_components):
            if stats[i,4] >= minSize:
                self.dst[output == i + 1] = 255

    def _findGateCenter(self):
        """
        Finds center of non-empty pixels and then extends to (hopefully) find the
        gate center. Note this uses the COLOR destination image.
        """
        return

        # check size of destination image 
        if np.shape(self.dst_color) != (self.y_res, self.x_res, 3):
            print "ERROR: incorrect image dimensions of {} for findCenter.".format(str(np.shape(self.dst_color)))
            return None

        # get center of all non-empty pixels
        y, x = np.rint(np.mean(np.nonzero(np.any(self.dst_color, axis=2)),axis=1))
        y = int(y)
        x = int(x)
        state_start = [y,x]

        # extend in each direction and find corresponding states
        state_top   = self._extendFromPoint(state_start, "UP")
        state_bot   = self._extendFromPoint(state_start, "DOWN")
        state_left  = self._extendFromPoint(state_start, "LEFT")
        state_right = self._extendFromPoint(state_start, "RIGHT")

        # find new mean state
        self.states = np.array([state_top, state_bot, state_left, state_right])

        # get final values
        y_center, x_center = np.mean(self.states, axis=0)

        self._gate_centerPixel_y = int(np.rint(y_center))
        self._gate_centerPixel_x = int(np.rint(x_center))

    def _getHoughIntersections(self):
        """
        Function to find the interseciton points of the hough lines. Clears the
        current hough intersection points if any exist.
        """
        # check that there are hough lines available
        if not(np.any(self._hough_lines)):
            print "ERROR: no hough lines exist for getHoughIntersections."
            return None

        # clear any existing hough intersections
        points = []
        check_list = np.array([[0,0]], dtype=np.int32)

        lines = self._hough_lines

        # there are smarter ways to do this but eh
        for line1 in self._hough_lines:
            for line2 in self._hough_lines:

                 # check if the line normals are 'different enough'
                 if not(line1.checkNormals(line2, tol=1.0)):

                    # if they are, get the intesection point then double check
                    # that you actually got a point back
                    int_point = line1.getIntersectionPoint(line2)
                    if int_point:

                        # round point to the nearest int and ensure it's unique
                        # before adding to the final list
                        int_point.roundToInt()
                        if not([int_point.x, int_point.y] in check_list.tolist()):
                            check_list = np.append(check_list, np.array([[int_point.x, int_point.y]], dtype=np.int32), axis=0)
                            points.append([int_point.x, int_point.y])

        self._hough_intersectionPoints_p = np.array(points, dtype=np.float32)

        if self.record_image:
            for p in self._hough_intersectionPoints_p:
                cv.circle(self.dst_color, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1)

        return True

    def _findGateCorners(self):
        """
        Finds corner pixels of the gate using 4-cluster k-means. Note this depends
        on hough line intersection points must exist for this. Will clear any current
        gate corner points.
        """

        # params for 'edge' cases haha
        tol = 25.0
        same_count = 0

        # check that there are hough lines available
        if self._hough_intersectionPoints_p.size == 0 or self._hough_intersectionPoints_p.shape[0] < 4:
            print "ERROR: insufficient hough intersection points exist for findGateCorners."
            return None
        
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret,label,center=cv.kmeans(self._hough_intersectionPoints_p, 4, None, criteria, 10, cv.KMEANS_PP_CENTERS)

        for point1 in center:
            for point2 in center:
                if np.any(point1 - point2):
                    if np.linalg.norm(point1 - point2) < tol:
                         same_count += 1

        if same_count:
            if same_count == 2:
            # probably only getting two corners
                ret,label,center=cv.kmeans(self._hough_intersectionPoints_p, 2, None, criteria, 10, cv.KMEANS_PP_CENTERS)
                #print "TWO CORNERS"
            #else:
            # maybe three corners?
                #print "THREE CORNERS"
                #ret,label,center=cv.kmeans(self._hough_intersectionPoints_p, 3, None, criteria, 10, cv.KMEANS_PP_CENTERS)
        
        self._gate_corners = np.float64(center)
        if self.record_image:
            for p in self._gate_corners:
                cv.circle(self.dst_color, (int(p[0]), int(p[1])), 5, (255, 0, 0), -1)

    def _extendFromPoint(self, state_start, direction):
        """
        Function to grow from a point on the image in a specified direction
        unti you hit a point that is not (0,0,0). Expects state_start to be
        the first point as [y,x] to align with image dimensions. Returns first
        non-zero point hit during growth as a list [y,x]. Note this uses the
        COLOR destination image.
        """

        # check for valid extension direction
        if not(direction in ["UP","DOWN","LEFT","RIGHT"]):
            print "ERROR: incorrect direction of {} for extendFromPoint.".format(direction)
            return None
    
        # set current state to y,x to align with image dimensions
        state_cur = state_start[:]
    
        # set grow direction as +1 or -1 and axis direction as 0 or 1
        # (i.e. y or x) based on direction argument
        grow_dir = 1 if (direction == "UP"   or direction == "RIGHT") else -1
        axis_dir = 1 if (direction == "LEFT" or direction == "RIGHT") else  0

        # extend until you hit a non-empty cell
        try:
            while not(np.any(self.dst_color[state_cur[0], state_cur[1], :])):
                state_cur[axis_dir] += grow_dir
        except IndexError as e:
            # Gone as far as we can
            pass
        return state_cur




def signal_handler(_, __):
    sys.exit(0)




def callback_img(self,data,args):
    global bridge
    global publisher_gate

    img = bridge.imgmsg_to_cv2(data)

    # upload src image to finder
    finder.updateSrc(src)

    # process src image
    finder.processSrc()

    # DEBUG: get pixel data for image display
    center_x = finder._gate_centerPixel_x
    center_y = finder._gate_centerPixel_y
    dst_cp = finder.dst_color[:]
    dst_cp[center_y-1:center_y+1, center_x-1:center_x+1, 1] = 255
    for p in finder._hough_intersectionPoints_p:
        cv2.circle(dst_cp, (int(p[0]), int(p[1])), 2, (0,255,0), -1)
    for p in finder._gate_corners:
        cv2.circle(dst_cp, (int(p[0]), int(p[1])), 5, (255,0,0), -1)
    

    img_msg = img = bridge.cv2_to_imgmsg(dst_cp)
    publisher_gate.publish(img_msg)




# if __name__ == '__main__':

#     signal.signal(signal.SIGINT, signal_handler)
#     rospy.init_node('image_processing', anonymous=False)

#     finder = gateFinder(1280, 720)

#     publisher_gate = rospy.Publisher('/visual/test_hough', Image, queue_size=1)
    
#     bridge = CvBridge()

    
#     rospy.Subscriber('/auto/gate_detection_active', Image, callback_img,1)
#     # rospy.Subscriber('/auto/gate_detection_active', Image, callback_img,1)

#     rospy.spin()


