import os
import pandas as pd
from PIL import Image

YOLO_LABELS_PATH = "../datasets/ArUco/labels"
VISANN_PATH = "../datasets/ArUco/annotations.txt"
VISIMG_PATH = "../datasets/ArUco/images"
split_train_val = 95  ## 118 images -> 95 train, others val

def convert(bbox, img_size):
    #将标注visDrone数据集标注转为yolov5
    #bbox top_left_x, down_right_x, top_left_y, down_right_y
    ## note the original point is on the top_left corner, and the direction of y-axis is down toward
    dw = 1/(img_size[0])
    dh = 1/(img_size[1])
    x = (bbox[0] + bbox[1])/2
    y = (bbox[2] + bbox[3])/2
    x = x * dw
    y = y * dh
    w = (bbox[1] - bbox[0]) * dw
    h = (bbox[3] - bbox[2]) * dh
    return (x,y,w,h) 

def ChangeToYolo5():
    if not os.path.exists(YOLO_LABELS_PATH):
        os.makedirs(YOLO_LABELS_PATH)

    count=0    ## label_idx
    ## read each line of annotation
    with open(VISANN_PATH, 'r') as f:
        for line in f.readlines():
            label_original = line.strip('\n')

            ## create labels
            for img_idx in range(1,10):
                image_path = VISIMG_PATH + '/scene'+str(count)+'_'+str(img_idx)+'.png'
                out_file = open(YOLO_LABELS_PATH + '/scene' + str(count)+'_'+str(img_idx)+'.txt', 'w')
                img = Image.open(image_path)
                img_size = img.size

                label = convert([int(x) for x in line.split()], img_size)
                out_file.write(str(1) + " " + " ".join(str(f'{x:.6f}') for x in label) + '\n')
                out_file.close()

            count += 1

if __name__ == '__main__':
    ChangeToYolo5()