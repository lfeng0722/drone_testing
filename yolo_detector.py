import os
import time
time.sleep(3)
import numpy as np
import torch

try:
    from models.experimental import attempt_load
    from utils.general import  (check_img_size, check_imshow, check_requirements,
                            check_suffix, colorstr, increment_path, non_max_suppression, print_args, save_one_box,
                            scale_coords, strip_optimizer, xyxy2xywh)
    from detect import parse_opt
    from utils.augmentations import letterbox
    from utils.plots import Annotator, Colors
except:
    from yolov5.models.experimental import attempt_load
    from yolov5.utils.general import  (check_img_size, check_imshow, check_requirements,
                            check_suffix, colorstr, increment_path, non_max_suppression, print_args, save_one_box,
                            scale_coords, strip_optimizer, xyxy2xywh)
    from yolov5.detect import parse_opt
    from yolov5.utils.augmentations import letterbox
    from yolov5.utils.plots import Annotator, Colors    
import cv2




def detect(model, img0, opt):
    stride = int(model.stride.max())  # model stride
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    img_size = opt.imgsz
    img_size = check_img_size(img_size, s=stride)  # check image size: check whether the original image size is the multiple of the stride, if not satisfy, a satisfied imgsz will be calculated automatically
    # cudnn.benchmark = True  # set True to speed up constant image size inference
    # Padded resize: resize images to 640*640
    img = letterbox(img0, img_size, stride=stride)[0]  ## img: the resized image

    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)  ## convert the incontinuous memory to continuous memory, to speed the computation speed
    img = torch.from_numpy(img).cuda()   ## e.g., img: torch.Size([3,640, 480]), 3 is channel, 640 is the height, 480 is the width of the image
    img = img.float()
    img /= 255  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim -> expand size [3, 640, 480] to [1, 3, 640, 480]
    
    pred = model(img, augment=False, visualize=False)[0]   ## e.g., torch.Size([1, 18908, 85]): 18908 bounding boxes are detected
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, opt.agnostic_nms, max_det=opt.max_det)  ## max_det is the default maximum bounding boxes
    # print(pred)

            # Process predictions
    for i, det in enumerate(pred):  # det is per image  ## size of each image is (5,6)
        # seen += 1   ## seen equal to counter
        # if webcam:  # batch_size >= 1
        #     p, im0, frame = path[i], im0s[i].copy(), dataset.count
        #     s += f'{i}: '
        # else:
        #     p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0) ## getattr: if no frame in dataset, return 0

        # p = Path(p)  # to Path
        # save_path = str('test_result.jpg')  # img.jpg
        # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
        # s += '%gx%g ' % img.shape[2:]  # print string
        # gn = torch.tensor(img.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        # imc = img0
        # imc = im0.copy() if save_crop else im0  # for save_crop -> to judge whether need to crop and save the detected rectangular
        annotator = Annotator(img0, line_width=5, example=str(names))   ## a tool for drawing, "name" is the label name (80 classes)
        if len(det):  ## this line of code is to judge whether there is at least one detected rectangular, if it exists, use the following code to draw it
            # Rescale boxes from img_size to im0 size
            # det[:, :4] = scale_coords(img0.shape[2:], det[:, :4], img0.shape).round()
            det[:, :4] = det[:, :4].round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                # s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string ## recode: e.g., 2 cars, 1 person, 2 cat

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
                # label =  f'{names[c]} {conf:.2f}'   ## hide or draw labels and the probability
                label = f'{conf:.2f}'
                colors = Colors()
                annotator.box_label(xyxy, label, color=colors(c, True))
                # if save_crop:
                    # save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

        # Print time (inference-only)
        # LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

        # Stream results
        im0 = annotator.result()
        # cv2.imshow('detect result', im0)
        # cv2.waitKey(1)  # 1 millisecond
        return pred, im0


def load_model():
    model = attempt_load('yolov5/runs/best_latest.pt')
    return model

if __name__ == "__main__":
    device = torch.device('cuda')
    opt = parse_opt()
    model = load_model()
    model = model.to(device)

    img = cv2.imread('datasets/images/scene0_1.png')
    detect(model, img, opt)

