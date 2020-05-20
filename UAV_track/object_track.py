# USAGE
# python opencv_object_tracking.py
# python opencv_object_tracking.py --video dashcam_boston.mp4 --tracker csrt

# import the necessary packages
import math

import numpy as np
import threading
import time
import cv2
import speedControl
import centroid__tracker

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,  # 比较精确，但是比kcf和mosse慢
    "kcf": cv2.TrackerKCF_create,  # 比csrt快，但精确度没有csrt高
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create  # 最快，但是精度不高
}

iLowH = 90
iHighH = 130
iLowS = 43
iHighS = 255
iLowV = 46
iHighV = 255

# 数字8：前进；5：悬停；2：后退；4：左；6：右
LAND = 48
LEFT_BACK = 49
BACK = 50
RIGHT_BACK = 51
LEFT = 52
HOVER = 53
RIGHT = 54
LEFT_GO = 55
GO = 56
RIGHT_GO = 57
saved_box = []

MANUAL = False
HOLD_ON = True
# 手动控制无人机的速度
SPEED = 50
# 紧急降落
EMERGENCE = False
# 记录小车的位置
cars = []
img_center = [0, 0]
# right, forward表示目前位置与目标位置的误差（像素差x0.1）
right = 0
forward = 0
offset_px = 0
offset_py = 0
frame = None

sendAdd2 = "AT+TXA=0x11,0x22,0x33,0x44,0x55".encode('utf-8')
sendAdd3 = "AT+TXA=0xAA,0xBB,0xCC,0xDD,0xEE".encode('utf-8')


def object_tracker(camID=0):
    global frame
    global offset_px, offset_py
    global forward, right
    global HOLD_ON, MANUAL, EMERGENCE
    # 记录目标的上一次的位置 [(id,(x,y))]
    lastCentroid = []
    # 记录目标ID
    ID = 0
    # 记录消失的帧数
    disappeared = []
    # 初始化multi_tracker
    trackers = cv2.MultiTracker_create()
    # 创建目标跟踪器
    ct = centroid__tracker.CentroidTracker()

    # if a video path was not supplied, grab the reference to the web cam
    # cap = VideoStream(src=0).start()
    cap = cv2.VideoCapture(camID)

    # while True:
    while cap.isOpened():
        # rects = []
        ret, frame = cap.read()
        if not ret:
            break

        if not MANUAL:
            # frame = imutils.resize(frame, width=600)
            img_center[0] = int(frame.shape[1] / 2)
            img_center[1] = int(frame.shape[0] / 2)
            # TODO:修改了tmp

            # 从trackers中获取正在跟踪的目标
            frame = cv2.resize(frame, (600, 450))

            (success, boxes) = trackers.update(frame)

            i = 100
            id = 0
            cnt1 = 0
            cnt2 = 0
            for box in boxes:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, i, 0), 2)
                cars[id] = (int((2 * x + w) / 2), int((2 * y + h) / 2))
                cnt1 = cnt1 + cars[id][0]
                cnt2 = cnt2 + cars[id][1]
                # rects.append(box)
                id = id + 1
                i = i + 50
            # 获取识别到的目标ID
            objects = ct.update(boxes)
            # 显示相应ID
            for (objectID, centroid) in objects.items():
                (lastX, lastY) = lastCentroid[objectID]
                a = (lastX - centroid[0]) ** 2 + (lastY - centroid[1]) ** 2
                if math.sqrt(a) < 5:
                    disappeared[objectID] += 1
                    lastCentroid[objectID] = centroid
                if disappeared[objectID] > 10:
                    pass
                text = "ID {}".format(objectID)
                cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

            # 显示车辆的中心位置和位置偏差
            if id > 0:
                cars_center = [int(cnt1 / id), int(cnt2 / id)]
                right = -cars_center[0] + img_center[0]  # 偏差值，单位是像素,位于中心右侧和上面时，偏差为负
                forward = cars_center[1] - img_center[1]

                # 当前位置与目标位置相差小于15个像素时，就认为已经到达了目标位置;或者如果HOLD_ON为真（表示还没有框住所有的目标）
                if right ** 2 + forward ** 2 < 225 or HOLD_ON:
                    right = forward = 0

                cv2.circle(frame, tuple(cars_center), 4, (255, 255, 0))
                # 画出图像中心点
                cv2.circle(frame, tuple(img_center), 4, (255, 0, 0))
                cv2.putText(frame, 'right=' + str(right) + ' pixels', (20, 40), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 1)
                cv2.putText(frame, 'forward=' + str(forward) + ' pixels', (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 1)
            else:
                right = forward = 0
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # 按下s选择目标
        if key == ord("s"):
            HOLD_ON = True
            if not MANUAL:
                # 按下s，用鼠标框住目标物，再按空格或回车
                box = cv2.selectROI("Frame", frame, fromCenter=False,
                                    showCrosshair=True)
                if any(box):
                    # 如果选中的区域存在，创建一个新目标
                    tracker = OPENCV_OBJECT_TRACKERS['csrt']()
                    trackers.add(tracker, frame, box)
                    (x, y, w, h) = box
                    cars.append(((2 * x + w) / 2, (2 * y + h) / 2))
                    lastCentroid.append((int((2 * x + w) / 2), int((2 * y + h) / 2)))
                    disappeared.append(0)
                print("Enter 'e' to start flying mission.")
        if key == ord("e") or MANUAL:
            HOLD_ON = False

        # 按下g获取此时的偏差
        if key == ord("g"):
            print("x=%.2f, y=%.2f" % (right, forward))
        elif key == GO:
            right = 0
            forward = SPEED
            MANUAL = True
            print("go")
        elif key == BACK:
            right = 0
            forward = -SPEED
            MANUAL = True
            print("back")
        elif key == LEFT:
            right = -SPEED
            forward = 0
            MANUAL = True
            print("LEFT")
        elif key == RIGHT:
            right = SPEED
            forward = 0
            MANUAL = True
            print("RIGHT")
        elif key == HOVER:
            right = 0
            forward = 0
            MANUAL = True
            print("HOVER")
        elif key == LEFT_GO:
            right = -SPEED
            forward = SPEED
            MANUAL = True
            print("LEFT_GO")
        elif key == LEFT_BACK:
            right = -SPEED
            forward = -SPEED
            MANUAL = True
            print("LEFT_BACK")
        elif key == RIGHT_BACK:
            right = SPEED
            forward = -SPEED
            MANUAL = True
            print("RIGHT_BACK")
        elif key == RIGHT_GO:
            right = SPEED
            forward = SPEED
            MANUAL = True
            print("RIGHT_GO")
        elif key == LAND:
            global EMERGENCE
            EMERGENCE = True
        # 按下q退出程序
        elif key == ord("q"):
            break
    # cap.stop()
    cap.release()
    cv2.destroyAllWindows()


def disappeared():
    maxDisappeared = 10


# 传入现有的目标及坐标、消失的目标ID
# 通过颜色识别得到的一些目标如果和传入的目标坐标很远，说明目标需要重新使用tracker
def colorRecognition(objects, id):
    PI = 3.14159
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsvSplit = cv2.split(imgHSV)
    hsvSplit[2] = cv2.equalizeHist(hsvSplit[2])
    imgHSV = cv2.merge(hsvSplit)

    imgThreshold = cv2.inRange(imgHSV, np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    imgThreshold = cv2.morphologyEx(imgThreshold, cv2.MORPH_OPEN, element)

    imgThreshold = cv2.morphologyEx(imgThreshold, cv2.MORPH_CLOSE, element)
    ret, imgThreshold = cv2.threshold(imgThreshold, 60, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(imgThreshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # img, contours, hierarchy = cv2.findContours(imgThreshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历所有轨迹，计算出其面积和外接圆的面积，两者相差最小的就是瓶盖
    tmp = np.zeros(frame.shape, np.uint8)
    chosenContours = []
    center = (0, 0)
    for i in range(len(contours)):
        epsilon = 0.1 * cv2.arcLength(contours[i], True)
        approx = cv2.approxPolyDP(contours[i], epsilon, True)
        # approx是拟合的原始轮廓
        (x, y), radius = cv2.minEnclosingCircle(approx)

        # 计算原轮廓的面积
        contour_area = cv2.contourArea(contours[i])
        circle_area = PI * radius * radius
        # 计算面积之比
        rate = (contour_area - circle_area) / contour_area
        if np.abs(rate) < 0.35:
            chosenContours.append(contours[i])
            color = (0, 0, 255)  # 被选中的是红色轮廓(opencv中颜色为BGR)
            thickness = 4
            center = (x, y)
        else:
            color = (255, 0, 0)  # 未被选中则显示蓝色轮廓
            thickness = 4
        # 画出外接圆
        # TODO:修改了tmp
        # SCI = cv2.circle(SCI, (int(x), int(y)), int(radius), color, thickness)
        # SCI = cv2.drawContours(SCI, contours, i, (0, 255, 0))  # 原始轮廓为绿色

    # 图像中心
    if len(chosenContours) > 0:
        return chosenContours


'''
返回无人机的目标位置
Return: x方向、y方向
'''

def get_offset():
    # right, forward表示目前位置与目标位置的误差（像素差x0.1）
    # global right, forward
    # global EMERGENCE
    scale = 0.1

    return scale * right, scale * forward, EMERGENCE


class DetectThread(threading.Thread):
    def __init__(self, name):
        super(DetectThread, self).__init__()
        self.name = name

    def run(self):
        object_tracker(0)


if __name__ == "__main__":
    # 下列代码演示了按下空格之后获得当前的摄像头图像的信息
    # 测试UAV能否可以到达视野中的目标：先悬浮一会，看到摄像头图中出现目标时，点击图像，按下空格键，无人机才开始飞向目标，同时记录此时的图像（或者位置偏差值）
    detectTarget_thread = DetectThread("Target Detect")
    detectTarget_thread.start()
    time.sleep(3)  # 要在子线程启动，显示出SCI之后才能在下面的while函数中显示SCI，否则没有这句话的话会报错

