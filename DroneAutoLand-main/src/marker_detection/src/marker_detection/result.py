#!/usr/bin/env python


from typing import Tuple, List, Union
import math

import numpy


class Position():
    def __init__(self, x:float = 0.0, y:float = 0.0, z:float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def set_from_vector(self, pv:numpy.ndarray):
        """3D Position as a vector in [x,y,z] order
        """
        if pv.size != 3:
            raise IndexError('Vector must be length 3: pv=%s' % str(pv.size))

        self.x = pv[0]
        self.y = pv[1]
        self.z = pv[2]


class Rotation():
    def __init__(self, w:float = 1.0, x:float = 0.0, y:float = 0.0, z:float = 0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def set_from_vector(self, qv:numpy.ndarray):
        """Quaternion as a vector in [w,x,y,z] order
        """
        if qv.size != 4:
            raise IndexError('Vector must be length 4')

        self.w = float(qv[0])
        self.x = float(qv[1])
        self.y = float(qv[2])
        self.z = float(qv[3])

    def set_from_r_vec(self, rv:numpy.ndarray):
        if rv.size != 3:
            raise IndexError('Vector must be length 3: rv=%s' % str(rv.size))

        # Pull out angle-axis representation then convert to quaternion
        # Essentially, length is angle and normalized vector is axis
        theta = math.sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2])
        st = math.sin(theta/2)

        self.w = math.cos(theta/2)
        self.x = float((rv[0]/theta) * st)
        self.y = float((rv[1]/theta) * st)
        self.z = float((rv[2]/theta) * st)


    def to_angle_axis(self):
        angle = 2 * math.acos(self.w)
        x = self.x / math.sqrt(1-self.w*self.w)
        y = self.y / math.sqrt(1-self.w*self.w)
        z = self.z / math.sqrt(1-self.w*self.w)

        return (angle, x, y, z)

class PoseResult():
    def __init__(self, id:int=0, position:Position = Position(), rotation:Rotation = Rotation(), confidence:float = 0.0, debug_image=None):
        """Result information a detected target:
        id: an identifier that uniquely specifies the detected target
        position: the position of the target relative to the camera
        rotation: the rotation of the target relative to the camera
        confidence: a confidence value based on the detection certainty
        """
        self.id = id
        self.position = position
        self.rotation = rotation
        self.confidence = float(confidence)
        self.debug_image = debug_image

class PixelResult():
    def __init__(self, marker_id=0, confidence=-1, min_x=0, min_y=0, max_x=0, max_y=0, debug_image=None):
        self.marker_id = marker_id
        self.confidence = confidence
        self.debug_image = debug_image

        min_x = int(min_x)
        max_x = int(max_x)
        min_y = int(min_y)
        max_y = int(max_y)
        self.marker_center = ((min_x + max_x) // 2, (min_y + max_y) // 2)
        self.marker_corners = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    
    def __str__(self):
        str_dict = {'marker_id': self.marker_id, 'confidence': self.confidence,
        'marker_center': self.marker_center,'marker_corners': self.marker_corners}
        return str(str_dict)

class PixelResults():
    def __init__(self, results:List[PixelResult]=[], debug_images:List[Union[numpy.ndarray,None]]=[]):
        """Results container for output data of a single processed image:
        results: a list of Result() containing information on a detected target
        debug_images: a list of debug images to transmit as required
        (pass 'None' if an indexed debug image does not exist for this frame -- e.g. [img, None, img])
        """
        self.results = results
        self.debug_images = debug_images
