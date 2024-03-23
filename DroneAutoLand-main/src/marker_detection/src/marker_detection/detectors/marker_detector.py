import numpy

class MarkerDetector(object):
    def __init__(self, name='base'):
        self.name = name
        self.type = 'ARUCO'
        self.encoding = 'bgr8'
    
    def shutdown(self):
        """Use to perform any required shutdown tasks or unloading before node is destroyed
        """
        pass


    def detect(self, image:numpy.ndarray, debug:bool=False):
        """This will be called along with the image to process, and the camera parameters for that image
        The 'debug' flag will specify if a debug image should be provided on return
        """
        results = []
        image_debug = None

        # Do processing here
        # image should be provided as be 'rgb8' encoding

        if debug:
            # Do something here
            pass

        return (results, [image_debug])