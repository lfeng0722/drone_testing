from marker_detection.detectors.marker_detector import MarkerDetector
from marker_detection.result import PixelResult
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
from marker_detection.tphyolo.models.experimental import attempt_load
from marker_detection.tphyolo.utils.general import (check_img_size, check_imshow, check_requirements,
                                  check_suffix, colorstr, increment_path, non_max_suppression, print_args, save_one_box,
                                  scale_coords, strip_optimizer, xyxy2xywh)
from marker_detection.tphyolo.utils.augmentations import letterbox
from marker_detection.tphyolo.utils.plots import Annotator, Colors
import numpy as np
import torch
import math
import cv2

class YoloOptions:
    def __init__(self):
        self.imgsz = [640, 640]
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.classes = None
        self.agnostic_nms = False
        self.max_det =  1000


class TphyoloDectector(MarkerDetector):
    def __init__(self, model_path, name='tphyolo_detector', device='cuda', target_marker_id=0):
        self.name = name
        self.target_marker_id = target_marker_id
        self.model = attempt_load(model_path)
        self.model.to(torch.device(device))
        self.detector_options = YoloOptions()
    

    def detect(self, image):
        stride = int(self.model.stride.max())  # model stride
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        img_size = self.detector_options.imgsz
        img_size = check_img_size(img_size,
                                s=stride)  # check image size: check whether the original image size is the multiple of the stride, if not satisfy, a satisfied imgsz will be calculated automatically
        img = letterbox(image, img_size, stride=stride)[0]  ## img: the resized image
        image_to_show = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(
            img)  ## convert the incontinuous memory to continuous memory, to speed the computation speed
        img = torch.from_numpy(
            img).cuda()  ## e.g., img: torch.Size([3,640, 480]), 3 is channel, 640 is the height, 480 is the width of the image
        img = img.float()
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]

        pred = self.model(img, augment=False, visualize=False)[
            0]  ## e.g., torch.Size([1, 18908, 85]): 18908 bounding boxes are detected
        pred = non_max_suppression(pred, self.detector_options.conf_thres, self.detector_options.iou_thres,
                                     self.detector_options.classes, self.detector_options.agnostic_nms,
                                max_det=self.detector_options.max_det)  ## max_det is the default maximum bounding boxes


        # Process predictions
        for i, det in enumerate(pred):  # det is per image  ## size of each image is (5,6)
            annotator = Annotator(image_to_show, line_width=5,
                                example=str(names))  ## a tool for drawing, "name" is the label name (80 classes)
            if len(det):  ## this line of code is to judge whether there is at least one detected rectangular, if it exists, use the following code to draw it
                det[:, :4] = det[:, :4].round()
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    # if conf < 0.8:
                    #     continue
                    # if save_txt:  # Write to file
                    #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #     line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                    #     with open(txt_path + '.txt', 'a') as f:
                    #         f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    # if save_img or save_crop or view_img:  # Add bbox to image
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'  ## hide or draw labels and the probability
                    colors = Colors()
                    annotator.box_label(xyxy, label, color=colors(c, True))


        # Stream results
        im0 = annotator.result()
        # cv2.imshow('detect result', im0)
        # cv2.waitKey(1)  # 1 millisecond

        pixel_result = None
        detected = False

        for result in pred[0]:
            # print(names)
            # print(result)
            marker_id = int(names[int(result[-1])][-1])
            confidence = result[-2]

            # print(marker_id)
            if marker_id == self.target_marker_id:
                # marker_center = ((result[0] + result[2]) // 2, (result[1] + result[3]) // 2)
                pixel_result = PixelResult(marker_id, confidence.item(), result[0].item(), result[1].item(),
                                             result[2].item(), result[3].item(), im0)
                detected = True
                break

        if not detected:
            pixel_result = PixelResult(debug_image=im0)        
            
        # print(pred, names)
        return pixel_result


    def set_target_marker(self, target_marker_id):
        self.target_marker_id = target_marker_id
