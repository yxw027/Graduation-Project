# 处理图像识别,图像的方向是无人机的后方
import cv2
import numpy as np
import serial.tools.list_ports
import math
import time
import speedControl
import threading

PI = 3.141592
iLowH = 90
iHighH = 130
iLowS = 43
iHighS = 255
iLowV = 46
iHighV = 255

# iLowH = 0
# iHighH = 180
# iLowS = 0
# iHighS = 30
# iLowV = 221
# iHighV = 255
pilot = np.zeros(2)
car2 = np.zeros(2)
car3 = np.zeros(2)
bias2 = np.zeros(2)
bias3 = np.zeros(2)
center = np.zeros(2)
img_center = np.zeros(2)
pf = 550  # 摄像头焦距（单位：像素）
HEIGHT = 1  # mm
right = 0
forward = 0
offset_px = 0
offset_py = 0
SCI = None

sendAdd2 = "AT+TXA=0x11,0x22,0x33,0x44,0x55".encode('utf-8')
sendAdd3 = "AT+TXA=0xAA,0xBB,0xCC,0xDD,0xEE".encode('utf-8')

def camInit(cameraID):
    cap = cv2.VideoCapture(cameraID)
    return cap

def colorDetect(cap):
    # hcom = serial.Serial("COM3", baudrate=115200, timeout=2)
    global offset_px, offset_py
    global forward, right
    global SCI
    i = 0
    while cap.isOpened():
        # time.sleep(0.05)
        # 读取帧摄像头
        ret, SCI = cap.read()  # 480x640 行数480
        SCI = cv2.resize(SCI, (600, 400))
        if ret:
            # TODO:删除了显示SCI，将其移动到了下方
            # cv2.imshow('Camera', SCI)
            cv2.waitKey(50)
            imgHSV = cv2.cvtColor(SCI, cv2.COLOR_BGR2HSV)
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
            tmp = np.zeros(SCI.shape, np.uint8)
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
                SCI = cv2.circle(SCI, (int(x), int(y)), int(radius), color, thickness)
                SCI = cv2.drawContours(SCI, contours, i, (0, 255, 0))  # 原始轮廓为绿色

            # 图像中心
            if len(chosenContours) > 0:

                img_center[0] = int(tmp.shape[1] / 2)
                img_center[1] = int(tmp.shape[0] / 2)
                # TODO:修改了tmp
                cv2.circle(SCI, (int(img_center[0]), int(img_center[1])), radius=10, color=(0, 255, 255))

                offset_px = -center[0] + img_center[0]  # 偏差值，单位是像素
                offset_py = center[1] - img_center[1]

            else:
                offset_py = 0
                offset_px = 0
            pos_right, pos_forward = speedControl.position_control(offset_px, offset_py)
            #TODO:tmp改为SCI
            cv2.putText(SCI, 'right=' + str(pos_right), (20, 40), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 1)
            cv2.putText(SCI, 'forward=' + str(pos_forward), (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 1)
            ## TODO:删除了显示tmp，改为显示SCI
            # cv2.imshow('Camera1', SCI)
        else:
            print("Can't open camera!")
            break

        if get_img():
            print(i)
            i = i + 1
    cap.release()
    cv2.destroyAllWindows()

def colorRecognition(SCI):
    cv2.waitKey(50)
    imgHSV = cv2.cvtColor(SCI, cv2.COLOR_BGR2HSV)
    hsvSplit = cv2.split(imgHSV)
    hsvSplit[2] = cv2.equalizeHist(hsvSplit[2])
    imgHSV = cv2.merge(hsvSplit)

    imgThreshold = cv2.inRange(imgHSV, np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    imgThreshold = cv2.morphologyEx(imgThreshold, cv2.MORPH_OPEN, element)

    imgThreshold = cv2.morphologyEx(imgThreshold, cv2.MORPH_CLOSE, element)
    ret, imgThreshold = cv2.threshold(imgThreshold, 60, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(imgThreshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历所有轨迹，计算出其面积和外接圆的面积，两者相差最小的就是瓶盖
    tmp = np.zeros(SCI.shape, np.uint8)
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
    # global offset_px, offset_py
    # offset_x = (offset_px * height / pf)
    # offset_y = (offset_py * height / pf)
    # print("px=%d,x=%.2f"%(offset_px, offset_x))
    # return offset_x, offset_y
    return right, forward

def get_img():
    cv2.imshow('camera', SCI)
    if cv2.waitKey(1) & 0xff == ord(' '):  # waitKey（0）似乎如果前面没有imshow()是没有用的
        right, forward = get_offset()
        print("x=%.2f, y=%.2f" % (right, forward))
        return True
    else:
        return False

class DetectThread(threading.Thread):
    def __init__(self, name):
        super(DetectThread, self).__init__()
        self.name = name

    def run(self):
        cap = camInit(0)
        colorDetect(cap)
        # test()


if __name__ == "__main__":
    # 下列代码演示了按下空格之后获得当前的摄像头图像的信息
    # 测试UAV能否可以到达视野中的目标：先悬浮一会，看到摄像头图中出现目标时，点击图像，按下空格键，无人机才开始飞向目标，同时记录此时的图像（或者位置偏差值）
    # detectTarget_thread = DetectThread("Target Detect")
    # detectTarget_thread.start()
    # time.sleep(3) # 要在子线程启动，显示出SCI之后才能在下面的while函数中显示SCI，否则没有这句话的话会报错
    # while True:
    #     if get_img():
    #         print("1")
    cap = camInit(0)
    colorDetect(cap)


