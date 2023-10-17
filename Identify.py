#-*-coding:utf-8-*-
# Function: Identify Treasure
#? 宝藏识别模块
#TODO Version 0.2.20230715
#! 依赖项目：OpenCV | numpy
import cv2
import numpy

global colorLow_Blue,colorhigh_Blue, colorLow_Green, colorhigh_Green, colorLow_Yellow, colorhigh_Yellow, colorLow_Red_0, colorhigh_Red_0, colorLow_Red_1, colorhigh_Red_1

colorLow_Blue = numpy.array([100, 43, 46])
colorHigh_Blue = numpy.array([124, 255, 255])

colorLow_Yellow = numpy.array([26, 43, 46])
colorHigh_Yellow = numpy.array([34, 255, 255])

colorLow_Green = numpy.array([34, 43, 30])
colorHigh_Green = numpy.array([77, 255, 255])

colorLow_Red_0 = numpy.array([0, 100, 50])
colorHigh_Red_0 = numpy.array([10, 255, 255])

colorLow_Red_1 = numpy.array([156, 100, 50])
colorHigh_Red_1 = numpy.array([180, 255, 255])



def reshape_image_scan(image):
    '''归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主'''
    width, height = image.shape[1], image.shape[0]
    min_len = width
    scale = width * 1.0 / 600
    new_width = 600

    new_height = int(height / scale)
    if new_height > 600:
        new_height = 600
        scale = height * 1.0 / 600
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height


def compute_center(contours, i):
    '''计算轮廓中心点'''
    M = cv2.moments(contours[i])  # 计算第一条轮廓的各阶矩,字典形式
    cx = int(M['m10'] / M['m00'])  # 计算轮廓中心点
    cy = int(M['m01'] / M['m00'])  # 计算轮廓中心点
    return cx, cy


def detecte(img):
    '''提取所有轮廓'''
    image = cv2.merge([img, img, img])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)  # 二值化
    contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)# 提取轮廓
    return image, contours, hierachy


def FindMaxOne(AllContours):
    max = 0
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[max]) and i != 0:
            # print("当前最大面积", cv2.contourArea(AllContours[i]))
            max = i
    return max


def FindSecondOne(AllContours):
    max = FindMaxOne(AllContours)
    second = 1
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[second]) and i != max and i != 0:

            second = i
    return second


def FindColorOne(frame,Color):  # (图片，颜色（0蓝,1黄,2绿,3红）)
    global colorLow_Blue, colorHigh_Blue, colorLow_Green, colorHigh_Green, colorLow_Yellow, colorHigh_Yellow, colorLow_Red_0, colorHigh_Red_0, colorLow_Red_1, colorHigh_Red_1

    frame = reshape_image_scan(frame)
    frame = frame[0]

    if Color == 0:
        colorLow_0 = colorLow_Blue
        colorHigh_0 = colorHigh_Blue
    elif Color == 1:
        colorLow_0 = colorLow_Yellow
        colorHigh_0 = colorHigh_Yellow
    elif Color == 2:
        colorLow_0 = colorLow_Green
        colorHigh_0 = colorHigh_Green
    elif Color == 3:
        colorLow_0 = colorLow_Red_0
        colorHigh_0 = colorHigh_Red_0
        colorLow_1 = colorLow_Red_1
        colorHigh_1 = colorHigh_Red_1

    # Show the original image.
    # cv2.imshow('frame', frame)

    # Blur methods available, comment or uncomment to try different blur methods.
    frameBGR = cv2.GaussianBlur(frame, (5, 5), 10)
    frameBGR = cv2.medianBlur(frameBGR, 7)  # 中值滤波
    # frameBGR = cv2.bilateralFilter(frameBGR, 15 ,75, 75) # 双边滤波
    # Show blurred image.
    # cv2.imshow('blurred', frameBGR)
    # HSV (Hue, Saturation, Value).
    # Convert the frame to HSV colour model.
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # HSV values to define a colour range.
    if Color == 0 or Color == 1 or Color == 2:
        mask = cv2.inRange(hsv, colorLow_0, colorHigh_0)
    elif Color == 3:
        mask0 = cv2.inRange(hsv, colorLow_0, colorHigh_0)
        mask1 = cv2.inRange(hsv, colorLow_1, colorHigh_1)
        mask = mask0 + mask1

    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    # Show morphological transformation mask
    image, contours, hierachy = detecte(mask)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)  # 画出轮廓, -1表示所有轮廓，（0,255,0）表示颜色，3表示线宽
    # cv2.imshow('counter', image)

    number = len(contours)
    # 计算面积小于100的轮廓数量
    for i in range(len(contours)):
        if cv2.contourArea(contours[i]) < 100:
            number = number - 1
    print("number", number)

    if Color == 0:
        # cv2.imshow('maskblue', mask)
        print("contours_blue", number)

    elif Color == 1:
        # cv2.imshow('maskyellow', mask)
        print("contours_yellow", number)

    elif Color == 2:
        # cv2.imshow('maskgreen', mask)
        print("contours_green", number)
    elif Color == 3:
        # cv2.imshow('maskred', mask)
        print("contours_red", number)
    # Put mask over top of the original image.
    # cv2.waitKey(0)

    # 如果轮廓数量小于2，返回原图, 不返回轮廓
    if number < 2:
        decision = 0
        return frame, contours, decision
    else:
        decision = 1
        return image, contours, decision




#定义形状检测函数
def ShapeDetection(copy_img,contours, Treasure):

    # 找到第二大的轮廓
    max = FindMaxOne(contours)
    second = FindSecondOne(contours)
    print("max:",max,"second:",second)
    # 获取第二大轮廓的坐标和长宽
    x, y, w, h = cv2.boundingRect(contours[second])
    cv2.rectangle(copy_img, (x, y), (x+w, y+h), (0, 0, 255), 2)  # 绘制边界框
    cv2.putText(copy_img, Treasure, (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)  # 绘制文字

