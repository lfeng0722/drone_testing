from marker_detection.detectors.marker_detector import MarkerDetector
from marker_detection.result import PixelResult
import cv2
from cv2 import aruco
import numpy


class OpenCVDetector(MarkerDetector):
    def __init__(self, name='opencv_detector', marker_dict=aruco.DICT_5X5_50, target_marker_id=0):

        self.name = name
        self.target_marker_id = target_marker_id
        self.type = 'fiducial'
        self.encoding = 'mono8'

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, image):
        """This will be called along with the image to process, and the camera parameters for that image
        The 'debug' flag will specify if a debug image should be provided on return
        """

        image_debug = None
        # corners, ids, _ = aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        corners, ids, _ = self.detector.detectMarkers(image)

        # print(corners, ids)

        # for i in range(len(self.boards)):
        #     board = self.boards[i]
            # n_found, r_vec, t_vec = aruco.estimateP
            # oseBoard(corners, ids, board, K, D, numpy.zeros((3,1)), numpy.zeros((3,1)))

        image_debug = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        detected = False
        if corners:
            for i in range(len(corners)):
                corner = corners[i][0]
                marker_id = ids[i][0]
                if marker_id == self.target_marker_id:

                    print('detected marker corners: ', corner)
                    confidence = 1
                    min_x = min([point[0] for point in corner])
                    max_x = max([point[0] for point in corner])
                    min_y = min([point[1] for point in corner])
                    max_y = max([point[1] for point in corner])
                    print('min_x: ', min_x, max_x, min_y, max_y)
                    pixel_result = PixelResult(marker_id, confidence, min_x, min_y,
                                        max_x, max_y, image_debug)
                    detected = True
                    break
            
        if not detected:
            pixel_result = PixelResult(debug_image=image_debug)

        return pixel_result

    def set_target_marker(self, target_marker_id):
        self.target_marker_id = target_marker_id

