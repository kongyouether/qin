# -*-coding:utf-8-*-
# Function: Main program
# ? 主程序
# TODO Version 0.2.20230715
# ! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py | networkx | itertools
# * Thread 利用情况：Thread-0 UART通信
# * Thread 利用情况：Thread-1 路径规划线程
# * Thread 利用情况：Thread-2 地图扫描线程
import sys, cv2, time, threading  # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton  # 导入PyQt5模块
from PyQt5 import QtGui, QtCore  # 导入PyQt5GUI模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours  # 导入地图寻路部分
from MapScan import reshape_image_scan, detect, find, affine_transformation, FindBlueOne  # 导入地图扫描部分
from Astar import Map, MapNode, astar, tsp  # 导入A*算法部分
import Identify  # 导入识别模块
import numpy as np  # 导入Numpy模块
import serialapi  # 导入串口模块

import networkx as nx
import itertools

import faulthandler

faulthandler.enable()

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720  # 设定的相机取样像素参数
result_final = []  # 寻路结果存储
True_Treas_Num = 0  # 已遍历真实宝藏点数目


def CreateStep(round, start, end, false):  # * 创建步进函数
    # round是轮次，0是起始轮，1是宝藏点中，2是最终路径，start是起点，end是终点，false是是否碰撞过了
    global result_final, boardmap, True_Treas_Num, treasureinmap, hitdirection, hitdistance
    print("round:", round, "start:", start, "end:", end, "false:", false)
    # TODO 将不作为起点和终点的宝藏点作为墙添加到boardmap中
    for i in range(0, len(treasureinmap)):
        if i != start and i != end:
            boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 0
            print("treasureinmap[i][1]:", treasureinmap[i][1], "treasureinmap[i][0]:", treasureinmap[i][0])
    if false == 1 and round != 0:
        boardmap[treasureinmap[start][1]][treasureinmap[start][0]] = 0

    # 将boardmap逐行打印出来
    for i in range(0, 19):
        print(boardmap[i])

    # TODO 计算此路段的行进方式
    # print("round:", round)
    if false == 1:  # 如果false为1，则进入不碰撞快速通过模式
        # 设置一个新的坐标，将宝藏点坐标存入其中
        address = treasureinmap[start]
        # 设置当前坐标为宝藏点坐标减去最后一步的方向和距离
        if (hitdirection == 0x00):
            address = [treasureinmap[start][0] - hitdistance * 2, treasureinmap[start][1]]
            print("上次方向0x00")
        elif (hitdirection == 0x01):
            address = [treasureinmap[start][0], treasureinmap[start][1] + hitdistance * 2]
            print("上次方向0x01")
        elif (hitdirection == 0x02):
            address = [treasureinmap[start][0] + hitdistance * 2, treasureinmap[start][1]]
            print("上次方向0x02")
        elif (hitdirection == 0x03):
            address = [treasureinmap[start][0], treasureinmap[start][1] - hitdistance * 2]
            print("上次方向0x03")
        print("address", address)
        if round == 2: # 如果是最后一段就直接去出口
            map = Map(boardmap, address[1], address[0], 0, 18, )
            print("next_destination:", 0, 18)
        else: # 如果不是最后一段就去下一个宝藏点
            map = Map(boardmap, address[1], address[0], treasureinmap[end][1], treasureinmap[end][0], )
            print("next_destination:", treasureinmap[end][0], treasureinmap[end][1])
        result_this_time = astar(map)
        result_this_time.reverse()


    else: # 如果false为0，则进入碰撞过后点到点模式
        if round == 0: # 如果是起始点就去第一个宝藏点
            map = Map(boardmap, 18, 0, treasureinmap[end][1], treasureinmap[end][0], )
            print("next_destination:", treasureinmap[end][0], treasureinmap[end][1])
            result_this_time = astar(map)
            result_this_time.reverse()
        elif round == 2: # 如果是最后一段就直接去出口
            map = Map(boardmap, treasureinmap[start][1], treasureinmap[start][0], 0, 18, )
            print("next_destination:", treasureinmap[start][1], treasureinmap[start][0])
            result_this_time = astar(map)
            result_this_time.reverse()
        else: # 如果不是起始点也不是最后一段就去下一个宝藏点
            map = Map(boardmap, treasureinmap[start][1], treasureinmap[start][0], treasureinmap[end][1], treasureinmap[end][0], )
            print("next_destination:", treasureinmap[end][1], treasureinmap[end][0])
            result_this_time = astar(map)
            result_this_time.reverse()

        # TODO 将boardmap恢复为原来的样子
    for i in range(0, len(treasureinmap)):
        if i != start and i != end:
            boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 1
            
    # step为一个30x2的二维数组，每一行代表一个数据帧，第一列代表功能码，第二列代表数据。
    # 功能码为0x00代表向北走，0x01代表向东走，0x02代表向南走，0x03代表向西走。
    # 北在地图右方，东在地图下方，南在地图左方，西在地图上方。

    # treasureinmap[itel][0]为y坐标，treasureinmap[itel][1]为x坐标

    # todo                  0x01↑
    # todo             0x02←       0x00→
    # todo                  0x03↓

    # 数据代表前进的步数，为0x00~0x64，即0~100。
    step = []
    for i in range(len(result_this_time) - 1):
        if (i != len(result_this_time) - 1):
            if (result_this_time[i][0] == result_this_time[i + 1][0]):
                if (result_this_time[i][1] == result_this_time[i + 1][1] + 1):
                    step.append([0x02, 0x01])
                elif (result_this_time[i][1] == result_this_time[i + 1][1] - 1):
                    step.append([0x00, 0x01])
            elif (result_this_time[i][1] == result_this_time[i + 1][1]):
                if (result_this_time[i][0] == result_this_time[i + 1][0] + 1):
                    step.append([0x01, 0x01])
                elif (result_this_time[i][0] == result_this_time[i + 1][0] - 1):
                    step.append([0x03, 0x01])
    # 将step整合，每次前进直到改变方向，即将连续的相同方向的步数相加
    i = 0
    while (i < len(step) - 1):
        if (step[i][0] == step[i + 1][0]):
            step[i + 1] = (step[i + 1][0], step[i][1] + step[i + 1][1])
            step.pop(i)
        else:
            i += 1
    # 设置一个新的列表step_new，将step中的数据存入其中，步数除以二
    step_new = []
    for i in range(len(step)):
        step_new.append([step[i][0], int(step[i][1] / 2)])
    # 将step最后一步的方向和距离存储到ifhit中
    print("step_new", step_new)
    hitdirection = step_new[-1][0]  # 方向
    hitdistance = step_new[-1][1]  # 距离

    # 将step_new最后一步减去
    # 如果这是最后一段路程，则步数加1
    if (round == 2):
        step_new[-1][1] += 1
    else:
        # 将step_new[-1]从step_new中删除
        step_new.pop(-1)
    for i in range(len(step_new)):
        if (step_new[i][0] == 0x00):
            print("→", step_new[i][1])
        elif (step_new[i][0] == 0x03):
            print("↓", step_new[i][1])
        elif (step_new[i][0] == 0x02):
            print("←", step_new[i][1])
        elif (step_new[i][0] == 0x01):
            print("↑", step_new[i][1])
    # 创建一个2xlength的矩阵step_final，将step中的数据存入其中
    length = len(step_new)
    step_final = np.zeros((2, length), dtype=np.uint8)
    for i in range(length):
        step_final[0][i] = step_new[i][0]
        step_final[1][i] = step_new[i][1]
    # 把step_final转换成1 x length * 2的数组
    step_final = step_final.reshape(1, length * 2)
    # print("step_final", step_final)
    return step_final, length * 2, result_this_time


'''  类封装  '''


class Camera:  # TODO 相机调取类封装

    def __init__(self, camera):
        self.frame = []
        self.ret = False
        self.cap = 0
        self.camera = camera

    def open(self):
        self.cap = cv2.VideoCapture(self.camera)
        self.ret = self.cap.set(3, CAMERA_WIDTH)
        self.ret = self.cap.set(4, CAMERA_HEIGHT)
        self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret = False
        # threading.Thread(target=self.queryframe, args=()).start() # Thread-1 启动进程，清除CV2缓存序列-暂停使用

    def queryframe(self):
        self.ret, self.frame = self.cap.read()

    def read(self):
        self.ret, self.frame = self.cap.read()
        return self.ret, self.frame  # 返回读取结果和帧  # ret是布尔值，frame是图像数组

    def release(self):
        self.cap.release()


class Example(QWidget):  # TODO 主窗口类

    def __init__(self):  # * 初始化函数
        super().__init__()

        self.initUI()

    def initUI(self):  # * 界面初始化函数

        self.lbl = QLabel('系统初始化进行中', self)
        self.lbl.move(0, 15)
        self.lbl.setFixedSize(1100, 45)
        self.lbl.setAlignment(QtCore.Qt.AlignCenter)  # 修改字体居中
        self.lbl.setFont(QtGui.QFont("Arial", 20))  # 修改字体大小为16px

        self.sublbl = QLabel(' 等待路径规划完成后运行', self)
        self.sublbl.move(0, 60)
        self.sublbl.setFixedSize(1100, 45)
        self.sublbl.setAlignment(QtCore.Qt.AlignCenter)  # 修改字体居中
        self.sublbl.setFont(QtGui.QFont("Arial", 16))  # 修改字体大小为12px

        # 拍照按钮
        self.stbtn = QPushButton('拍摄藏宝图', self)
        self.stbtn.clicked.connect(self.startup_thread)  # 修改按钮行为为启动特定函数“Startup”
        self.stbtn.move(72, 120)
        self.stbtn.setFixedSize(288, 75)
        # 设置按钮样式
        self.stbtn.setStyleSheet("QPushButton:hover{color:black}"  # 光标移动到上面后的前景色
                                 "QPushButton:hover{background-color:rgb(200,200,200)}"  # 光标移动到上面后的背景色
                                 # 按钮被禁用后的样式
                                 "QPushButton:disabled{color:black}"
                                 "QPushButton:disabled:hover{color:black}"
                                 "QPushButton:disabled{background-color:rgb(128,128,128)}"
                                 "QPushButton:disabled{border:0px}"
                                 "QPushButton:disabled{border-radius:20px}"
                                 "QPushButton:disabled{padding:2px 4px}"
                                 "QPushButton:disabled{font-family:'黑体';font-size:30px}"
                                 # 按钮正常显示样式
                                 "QPushButton{color:black}"  # 按键前景色
                                 "QPushButton{background-color:rgb(240,240,240)}"  # 按键背景色
                                 "QPushButton{border:0px}"  # 边界宽度
                                 "QPushButton{border-radius:20px}"  # 边界圆角半径
                                 "QPushButton{padding:2px 4px}"  # 按键内边界
                                 "QPushButton{font-family:'黑体';font-size:30px}")  # 字体样式
        # 调试时不禁用
        # self.stbtn.setEnabled(False) # 禁用按钮
        # 运行按钮
        self.runbtn = QPushButton('小车·启动', self)
        self.runbtn.clicked.connect(self.runcar_thread)  # 修改按钮行为为启动小车运行函数
        self.runbtn.move(396, 120)
        self.runbtn.setFixedSize(324, 75)
        self.runbtn.setStyleSheet("QPushButton:hover{color:black}"  # 光标移动到上面后的前景色
                                  "QPushButton:hover{background-color:rgb(200,200,200)}"  # 光标移动到上面后的背景色
                                  # 按钮被禁用后的样式
                                  "QPushButton:disabled{color:black}"
                                  "QPushButton:disabled:hover{color:black}"
                                  "QPushButton:disabled{background-color:rgb(128,128,128)}"
                                  "QPushButton:disabled{border:0px}"
                                  "QPushButton:disabled{border-radius:20px}"
                                  "QPushButton:disabled{padding:2px 4px}"
                                  "QPushButton:disabled{font-family:'黑体';font-size:30px}"
                                  # 按钮正常显示样式
                                  "QPushButton{color:black}"  # 按键前景色
                                  "QPushButton{background-color:rgb(240,240,240)}"  # 按键背景色
                                  "QPushButton{border:0px}"  # 边界宽度
                                  "QPushButton{border-radius:20px}"  # 边界圆角半径
                                  "QPushButton{padding:2px 4px}"  # 按键内边界
                                  "QPushButton{font-family:'黑体';font-size:30px}")  # 字体样式
        self.runbtn.setEnabled(False)  # 禁用按钮

        # 选择红色方
        self.redbtn = QPushButton('红方', self)
        self.redbtn.clicked.connect(self.red_thread)  # 修改按钮行为为启动红方线程
        self.redbtn.move(27, 330)
        self.redbtn.setFixedSize(180, 150)
        # 设置按钮样式
        self.redbtn.setStyleSheet("QPushButton:hover{color:black}"  # 光标移动到上面后的前景色
                                  "QPushButton:hover{background-color:rgb(238,0,0)}"  # 光标移动到上面后的背景色
                                  # 按钮被禁用后的样式
                                  "QPushButton:disabled{color:black}"
                                  "QPushButton:disabled:hover{color:black}"
                                  "QPushButton:disabled{background-color:rgb(128,128,128)}"
                                  "QPushButton:disabled{border:0px}"
                                  "QPushButton:disabled{border-radius:20px}"
                                  "QPushButton:disabled{padding:2px 4px}"
                                  "QPushButton:disabled{font-family:'黑体';font-size:35px}"
                                  # 按钮正常显示样式
                                  "QPushButton{color:black}"  # 按键前景色
                                  "QPushButton{background-color:rgb(190,0,0)}"  # 按键背景色
                                  "QPushButton{border:0px}"  # 边界宽度
                                  "QPushButton{border-radius:20px}"  # 边界圆角半径
                                  "QPushButton{padding:2px 4px}"  # 按键内边界
                                  "QPushButton{font-family:'黑体';font-size:35px}")  # 字体样式
        # 选择蓝色方
        self.bluebtn = QPushButton('蓝方', self)
        self.bluebtn.clicked.connect(self.blue_thread)
        self.bluebtn.move(873, 330)
        self.bluebtn.setFixedSize(180, 150)
        # 设置按钮样式
        self.bluebtn.setStyleSheet(
            "QPushButton:hover{color:black}"  # 光标移动到上面后的前景色
            "QPushButton:hover{background-color:rgb(102,204,255)}"  # 光标移动到上面后的背景色
            # 按钮被禁用后的样式
            "QPushButton:disabled{color:black}"
            "QPushButton:disabled:hover{color:black}"
            "QPushButton:disabled{background-color:rgb(128,128,128)}"
            "QPushButton:disabled{border:0px}"
            "QPushButton:disabled{border-radius:20px}"
            "QPushButton:disabled{padding:2px 4px}"
            "QPushButton:disabled{font-family:'黑体';font-size:35px}"
            # 按钮被启用后的样式
            "QPushButton{color:black}"  # 按键前景色
            "QPushButton{background-color:rgb(81,163,204)}"  # 按键背景色
            "QPushButton{border:0px}"  # 边界宽度
            "QPushButton{border-radius:20px}"  # 边界圆角半径
            "QPushButton{padding:10px 10px}"  # 按键内边界
            "QPushButton{font-family:'黑体';font-size:35px}")  # 字体样式

        # 退出按钮
        self.btn = QPushButton('断点暂停', self)
        self.btn.clicked.connect(self.stop_thread)  # 修改按钮行为为启动特定函数“stop”
        self.btn.move(756, 120)
        self.btn.setFixedSize(252, 75)
        self.btn.setStyleSheet("QPushButton:hover{color:black}"  # 光标移动到上面后的前景色
                               "QPushButton:hover{background-color:rgb(200,200,200)}"  # 光标移动到上面后的背景色
                               # 按钮被禁用后的样式
                               "QPushButton:disabled{color:black}"
                               "QPushButton:disabled:hover{color:black}"
                               "QPushButton:disabled{background-color:rgb(128,128,128)}"
                               "QPushButton:disabled{border:0px}"
                               "QPushButton:disabled{border-radius:20px}"
                               "QPushButton:disabled{padding:2px 4px}"
                               "QPushButton:disabled{font-family:'黑体';font-size:30px}"
                               # 按钮正常显示样式
                               "QPushButton{color:black}"  # 按键前景色
                               "QPushButton{background-color:rgb(240,240,240)}"  # 按键背景色
                               "QPushButton{border:0px}"  # 边界宽度
                               "QPushButton{border-radius:20px}"  # 边界圆角半径
                               "QPushButton{padding:2px 4px}"  # 按键内边界
                               "QPushButton{font-family:'黑体';font-size:30px}")  # 字体样式
        self.setGeometry(0, 0, 1100, 600)
        # self.setFixedSize(600, 400)
        self.setWindowTitle('宝藏程序')
        self.setFont(QtGui.QFont("Arial", 16))  # 修改字体大小为16px

        # 添加一个用于显示相机图像的控件
        self.camera_label = QLabel(self)
        self.camera_label.move(234, 225)
        self.camera_label.setFixedSize(612, 360)
        self.camera_label.setScaledContents(True)
        # 加载一张图片
        pixmap = QtGui.QPixmap('test.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)
        self.show()

        # 系统串口检测
        self.lbl.setText('检测串口通信通路')
        if serialapi.If_Serial_Open == False:  # 检测串口是否打开
            self.lbl.setText('串口通信通路异常,系统初始化失败')
        else:
            threading.Thread(target=serialapi.uartRx, args=()).start()  # * Thread-0 开启串口接收子线程
            # # 创建一个长度为60的一维数组test，元素初始值为0
            # test = np.zeros(60)
            # # 将test每个元素转换为16进制
            # test = test.astype(np.uint8)
            # # serialapi.communicate(0x01,0x0c,test,0x00,0x04) # 发送启动信号
            # serialapi.communicate(0x01, 0x05, [[0x01, 0x02, 0x03, 0x04, 0x05]], 0x00, 0x01)
            while True:
                # if str(serialapi.recv) == '000000ff':
                self.lbl.setText('串口通信通路正常,系统初始化完成')
                break
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.lbl.setText('串口通信通路正常,系统初始化完成')

    def red_thread(self):  # * 启动红方线程
        t = threading.Thread(target=self.red)
        t.start()

    def blue_thread(self):  # * 启动蓝方线程
        t = threading.Thread(target=self.blue)
        t.start()

    def startup_thread(self):  # * 启动路径规划线程
        t = threading.Thread(target=self.Startup)
        t.start()

    def runcar_thread(self):  # * 启动运行线程
        t = threading.Thread(target=self.Runcar)
        t.start()

    
    def stop_thread(self):  # * 启动暂停线程
        t = threading.Thread(target=self.Stop)
        t.start()


    def red(self):  # * 红方线程
        global set_team_color
        set_team_color = 0
        self.redbtn.setEnabled(False)
        self.bluebtn.setEnabled(False)
        self.stbtn.setEnabled(True)
        print("set_team_color:", set_team_color)

    def blue(self):  # * 蓝方线程
        global set_team_color
        set_team_color = 1
        self.redbtn.setEnabled(False)
        self.bluebtn.setEnabled(False)
        self.stbtn.setEnabled(True)
        print("set_team_color:", set_team_color)
    

    def Stop(self):  # * 暂停函数
        global TreasureIfo, ShouldStop
        ShouldStop = 1
        print("暂停")
        

    def Startup(self):  # * 启动函数
        global result_final, boardmap, map_gui_image, cell, map_cv2_image, treasureinmap, TreasureIfo, Trea_distances, cropimg, NextTreasure, First_distances, ShouldStop
        sys.setrecursionlimit(10000)
        # camera = Camera(1)  # 打开相机
        # camera.open()
        # # 等待调整相机并拍照
        # last_image = None
        # start_time = time.time()
        # scan_time = 3
        # while time.time() - start_time < scan_time:
        #     ret, image = camera.read()  # 读取相机图像
        #     # 将相机图像显示在控件中
        #     # 将BGR格式转换为RGB格式
        #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #     qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0], image.shape[1] * 3,
        #                         QtGui.QImage.Format_RGB888)
        #     pixmap = QtGui.QPixmap.fromImage(qimg)
        #     self.camera_label.setPixmap(pixmap)
        #     self.lbl.setText('调节时间剩余' + str(int(scan_time - time.time() + start_time)) + "秒，请注意")
        #     last_image = image
        #
        # camera.release()  # 释放相机资源1
        # 将图像保存到本地
        # cv2.imwrite('./mapcam.jpg', last_image)
        last_image = cv2.imread('./mapcam.jpg')  # 读取test.jpg

        # ? 照片扫描加纠偏部分开始
        start_time = time.time()
        self.lbl.setText('照片扫描纠偏中...')
        # image, new_width, new_height = reshape_image_scan(last_image)
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image, contours, hierachy = detect(last_image)
        rec, img = find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        
        xblue, yblue = FindBlueOne(img)
        # 如果蓝色色块在左上角或者右下角，就旋转90度
        if xblue < 400 and yblue < 400 or xblue > 400 and yblue > 400:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * 3, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # ? 照片扫描加纠偏部分结束

        # ? 地图扫描输出数组部分开始
        self.lbl.setText('地图扫描输出进行中...')
        img, width, height = reshape_image_find(img)  # 调整图片大小
        _, Binary_img = ToBinray(img)  # 转二进制
        contours, boardx, cropimg, cell, treasureinmap = GetGontours(img,Binary_img)  # 提取轮廓

        for i in range(len(boardx)):  # 将board中255的值转换为1
            for j in range(len(boardx[0])):
                if (boardx[i][j] == 255):
                    boardx[i][j] = 1
        boardmap = tuple(boardx)  # 将board转换为元组
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
                            QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # ? 地图扫描输出数组部分结束First_distances

        self.lbl.setText('路径规划进行中...')
        # ? 路径规划部分开始
        treasureinmap, Trea_distances, First_distances= tsp(boardmap, treasureinmap)  # 调用tsp算法
        # 创建一个新的数组TreasureIfo,用于存储宝藏信息
        # [0]为y坐标，[1]为x坐标，[2]为所处象限，[3]为镜像对称点, [4]为颜色信息, [5]为激活信息, [6]真假信息
        # 颜色信息：0代表未知,1代表红色，2代表蓝色
        # 激活信息：0代表未激活，1代表激活
        # 真假信息：0代表未知，1代表真，2代表假
        TreasureIfo = np.zeros((8, 7), dtype=np.uint8)
        for i in range(8):
            TreasureIfo[i][0] = treasureinmap[i][0]
            TreasureIfo[i][1] = treasureinmap[i][1]
            if (treasureinmap[i][0] > 9):
                if (treasureinmap[i][1] > 9):
                    TreasureIfo[i][2] = 3
                else:
                    TreasureIfo[i][2] = 2
            else:
                if (treasureinmap[i][1] > 9):
                    TreasureIfo[i][2] = 4
                else:
                    TreasureIfo[i][2] = 1
            TreasureIfo[i][4] = 0 # 颜色信息：0代表未知,1代表红色，2代表蓝色
            TreasureIfo[i][5] = 1 # 激活信息：0代表未激活，1代表激活
            TreasureIfo[i][6] = 0 # 真假信息：0代表未知，1代表真，2代表假
        # 如果两个宝藏点的x和y坐标相加和为18，则它们是镜像对称的,并在TreasureIfo[i][3]中标记
        k = 1
        for i in range(8):
            for j in range(i + 1, 8):
                if (TreasureIfo[i][0] + TreasureIfo[j][0] == 18 and TreasureIfo[i][1] + TreasureIfo[j][1] == 18):
                    TreasureIfo[i][3] = k
                    TreasureIfo[j][3] = k
                    k += 1
        print("TreasureIfo:", TreasureIfo)
        
        # ? 路径规划部分结束
        end_time = time.time()
        print("路径规划时间:", end_time - start_time)
        result_final = []
        for j in range(-1, 8, 1):
            if j == -1:
                map = Map(boardmap, 18, 0, treasureinmap[j + 1][1], treasureinmap[j + 1][0], )
            elif j == 7:
                map = Map(boardmap, treasureinmap[j][1], treasureinmap[j][0], 0, 18, )
            else:
                map = Map(boardmap, treasureinmap[j][1], treasureinmap[j][0], treasureinmap[j + 1][1],
                          treasureinmap[j + 1][0], )  # (mapdata,startx,starty,endx,endy)
            result = astar(map)
            result.reverse()
            resultx = np.asarray(result)
            result_final.append(result)
        # print("怎么移动：", result_final)
        # print(result_final[0][0][0], result_final[0][0][1])
        NextTreasure = 0 # 下一个宝藏点是第0个
        ShouldStop = 0 # 是否停止
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
                            QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # # 界面更新
        self.stbtn.setEnabled(False)
        self.runbtn.setEnabled(True)
        self.lbl.setText('路径规划完成,请按下Enter运行...')

    def Runcar(self):  # * 运行函数
        global  boardmap, True_Treas_Num, treasureinmap, ifhit, hitdistance, hitdirection, TreasureIfo, NextTreasure, Trea_distances, cropimg, ShouldStop, First_distances
        # 数据初始化
        print("运行")
        # True_Treas_Num = 0
        ifhit = 1
        CurrentTreasure = -1

        # 如果仍然处于停止状态，进行特殊的初始化
        # 将距离起点最近的激活点作为下一个宝藏点
        if ShouldStop == 1:
            min = 100
            for i in range(8):
                if TreasureIfo[i][5] == 1:
                    if First_distances[i] < min:
                        min = First_distances[i]
                        NextTreasure = i
            ShouldStop = 0 # 重置停止状态
            print("重置停止状态，断点重启初始化完成")

        # TODO 运行10段路程，完成宝藏遍历
        for itel in range(10):
            self.lbl.setText('计算...去第' + str(NextTreasure) + '个宝藏点的路')
            if ShouldStop == 1:
                # 结束当前进程
                break

            # TODO 计算下一段路程的行进方式
            Active_Num = 0
            for i in range(8):# 计算仍然激活的宝藏点的数目
                if TreasureIfo[i][5] == 1:
                    Active_Num += 1
            
            # 开始计时
            Astar_start_time = time.time()

            #! 如果上次没有击打
            if ifhit == 0:  
                if Active_Num != 0:# 如果还有激活的宝藏点
                    step_final, length, coord = CreateStep(1, CurrentTreasure, NextTreasure, 1)
                    print("刚才没击打，计算去下一个")
                else: # 如果没有激活的宝藏点
                    step_final, length, coord= CreateStep(2, CurrentTreasure, 999         , 1)
                    print("刚才没击打，计算去终点")
            #! 如果上次击打了
            else:
                if CurrentTreasure == -1: # 如果是第一个宝藏点
                    step_final, length, coord = CreateStep(0, 999            , NextTreasure, 0)
                    print("第一个，直接去第一个") 
                else:    
                    if Active_Num != 0: # 如果还有激活的宝藏点
                        step_final, length, coord = CreateStep(1, CurrentTreasure, NextTreasure, 0)
                        print("刚才击打了，计算去下一个")
                    else:  # 否则计算到终点的路程
                        step_final, length, coord = CreateStep(2, CurrentTreasure,999          , 0)
                        print("刚才击打了，计算去终点")
            # 结束计时
            Astar_end_time = time.time()
            print("计算Astar时间:", Astar_end_time - Astar_start_time)
            # 按照坐标路径在地图上绘制线段并显示
            for i in range(len(coord)):
                if i != 0:
                    cv2.line(cropimg, (cell * coord[i][1] + int(cell * 3), cell * coord[i][0] + int(cell * 1)),
                             (cell * coord[i - 1][1] + int(cell * 3), cell * coord[i - 1][0] + int(cell * 1)),
                             (0, 0, 255), 2)
                cv2.circle(cropimg, (cell * coord[i][1] + int(cell * 3), cell * coord[i][0] + int(cell * 1)), 2,
                           (100, 100, 100), 2)
            # 将图像显示在QT控件中
            # qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
            #                     QtGui.QImage.Format_RGB888)
            # pixmap = QtGui.QPixmap.fromImage(qimg)
            # self.camera_label.setPixmap(pixmap)
            # TODO 如果撞击了,等待撞击运行完成 
            if ifhit == 1 and CurrentTreasure != -1:
                #!!!!因调试注释掉：
                time.sleep(3)
                while True:
                    # if str(serialapi.recv) == '000000ff':
                    break  # 等待接收到回复信号
                    # print("等待撞击运行完成")
                    if ShouldStop == 1: break; # 如果停止了，结束当前进程
                serialapi.recv = str.encode('xxxxxxxxxxx')
                if ShouldStop == 1: break; # 如果停止了，结束当前进程
                print("撞击运行完成")
            
            # TODO 使用serialapi.communicate将step发给下位机
            while True:
                serialapi.communicate(0x01, eval(hex(length)), step_final, hitdirection, eval(hex(hitdistance)))
                time.sleep(0.05)
                 #!!!!因调试注释掉：
                time.sleep(1)
                # if str(serialapi.recv) == '000100ff':
                break# 等待接收到回复信号
                print("等待下位机接收完成")
                if ShouldStop == 1: break; # 如果停止了，结束当前进程
            serialapi.recv = str.encode('xxxxxxxxxxx')
            if ShouldStop == 1: break; # 如果停止了，结束当前进程
            self.lbl.setText('运行中...前往宝藏点')
            self.sublbl.setText("前往宝藏点" + str(NextTreasure))
            if Active_Num != 0:
                print("前往宝藏点", str(NextTreasure),"坐标为：", TreasureIfo[NextTreasure][0], TreasureIfo[NextTreasure][1])
            else:
                print("前往终点")

            # TODO 等待运行完成
            #!!!!因调试注释掉：
            time.sleep(2)
            while True:
                # if str(serialapi.recv) == '000000ff':
                break;  # 等待接收到回复信号
                if ShouldStop == 1: break; # 如果停止了，结束当前进程
            serialapi.recv = str.encode('xxxxxxxxxxx')
            if ShouldStop == 1: break; # 如果停止了，结束当前进程
            
            # 如果到达终点则结束
            if Active_Num == 0:
                self.lbl.setText('运行结束')
                self.sublbl.setText('运行结束')
                print("运行结束")
                break
            else:
                self.lbl.setText('已到达')
                self.sublbl.setText("第" + str(NextTreasure) + "个宝藏点已到达")
                print(TreasureIfo[NextTreasure][0], TreasureIfo[NextTreasure][1],"宝藏点已到达")
            # TODO 拍照识别
            # 如果宝藏的颜色信息和真假信息都未知，则进行拍照识别
            if TreasureIfo[NextTreasure][4] == 0 or TreasureIfo[NextTreasure][6] == 0:
                # open_camera_time = time.time()
                # camera = Camera(1) # 打开相机
                # camera.open()
                # start_time = time.time()
                # while time.time() - start_time < 0.5:
                #     ret, Treas_image = camera.read() # 读取相机宝藏图像
                #     # 将图片由BGR格式转换为RGB格式
                #     # Treas_image_RGB = cv2.cvtColor(Treas_image, cv2.COLOR_BGR2RGB)
                #     # Treas_qimg = QtGui.QImage(Treas_image_RGB.data, Treas_image_RGB.shape[1], Treas_image_RGB.shape[0],Treas_image_RGB.shape[1]*3, QtGui.QImage.Format_RGB888)
                #     # Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
                #     # self.camera_label.setPixmap(Treas_pixmap)
                #     self.sublbl.setText('剩余'+str(int(0.5-time.time() + start_time))+"秒进行拍摄识别")
                # camera.release()  # 释放相机资源
                # 开始计时
                # start_time = time.time()
                # distances = hitdistance  # 距离列表
                # # 将图像存在当前文件夹下,根据当前的坐标命名
                # cv2.imwrite('./' + str(TreasureIfo[NextTreasure][0]) + str(TreasureIfo[NextTreasure][1]) + '.jpg', Treas_image)
                # if distances == 1:
                #     print("距离为1的裁剪")
                #     Treas_image = Treas_image[int(Treas_image.shape[0] / 10) * 3:int(Treas_image.shape[0]/10) * 8, int(Treas_image.shape[1] / 10) * 1:int(Treas_image.shape[1] / 10) * 6]
                # elif distances == 2:
                #     print("距离为2的裁剪")
                #     Treas_image = Treas_image[int(Treas_image.shape[0] / 10) * 3:int(Treas_image.shape[0] / 10) * 7, int(Treas_image.shape[1] / 10) * 3:int(Treas_image.shape[1] / 10) * 6]
                # elif distances == 3:
                #     print("距离为3的裁剪")
                #     Treas_image = Treas_image[int(Treas_image.shape[0] / 10) * 2:int(Treas_image.shape[0] / 10) * 6, int(Treas_image.shape[1] / 10) * 3:int(Treas_image.shape[1] / 10) * 6]
                # # Treas_img_copy = Treas_image.copy()
                #
                # #! 因调试注释掉：
                # # Treas_image = cv2.imread('./redtrue.jpg')  # 读取test.jpg
                #
                #
                # Treas_image = reshape_image_scan(Treas_image)[0]
                # Treas_img_copy = Treas_image
                #
                # Treas_image, contours, yellow = Identify.FindColorOne(Treas_img_copy, 1,hitdistance)  # 黄色
                # Treas_image, contours, green = Identify.FindColorOne(Treas_img_copy, 2,hitdistance)  # 绿色
                # Treas_image, contours_blue, blue = Identify.FindColorOne(Treas_img_copy, 0, hitdistance)  # 蓝色
                # Treas_image, contours_red, red = Identify.FindColorOne(Treas_img_copy, 3, hitdistance)  # 红色
                #蓝色1，黄色1，为蓝色真宝藏；红色1，绿色1，为红色真宝藏；蓝色1，绿色1，为蓝色假宝藏；红色1，黄色1，为红色假宝藏
                print("拍照完毕")
                # 走流程识别，固定为红色真宝藏
                blue = 0
                yellow = 0
                red = 1
                green = 1
                # identify_end_time = time.time()
                # print("拍照识别时间:", identify_end_time - open_camera_time)
            else:
                blue = 0
                yellow = 0
                red = 0
                green = 0
                print("已知宝藏颜色，不进行拍照识别")
                
            if (blue == 1 and yellow == 1) or (TreasureIfo[NextTreasure][4] == 2 and TreasureIfo[NextTreasure][6] == 1):
                self.lbl.setText('蓝色真宝藏')
                self.sublbl.setText('蓝色真宝藏')
                print("蓝色真宝藏")
                Treasure = "BlueTrue"
            elif (red == 1 and green == 1) or (TreasureIfo[NextTreasure][4] == 1 and TreasureIfo[NextTreasure][6] == 1):
                self.lbl.setText('红色真宝藏')
                self.sublbl.setText('红色真宝藏')
                print("红色真宝藏")
                Treasure = "RedTrue"
            elif (blue == 1 and green == 1) or (TreasureIfo[NextTreasure][4] == 2 and TreasureIfo[NextTreasure][6] == 2):
                self.lbl.setText('蓝色假宝藏')
                self.sublbl.setText('蓝色假宝藏')
                print("蓝色假宝藏")
                Treasure = "BlueFalse"
            elif (red == 1 and yellow == 1) or (TreasureIfo[NextTreasure][4] == 1 and TreasureIfo[NextTreasure][6] == 2):
                self.lbl.setText('红色假宝藏')
                self.sublbl.setText('红色假宝藏')
                print("红色假宝藏")
                Treasure = "RedFalse"
            else:
                self.sublbl.setText('无宝藏')
                print("无宝藏")
                Treasure = "None"

            # 结束计时
            # 将图像由BGR格式转换为RGB格式
            # Treas_img_RGB = cv2.cvtColor(Treas_img_copy, cv2.COLOR_BGR2RGB)
            # Treas_qimg = QtGui.QImage(Treas_img_RGB.data, Treas_img_RGB.shape[1], Treas_img_RGB.shape[0],Treas_img_RGB.shape[1]*3, QtGui.QImage.Format_RGB888)
            # Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
            # self.camera_label.setPixmap(Treas_pixmap)
            # time.sleep(5)

            # TODO 计算宝藏信息
            #? 如果是蓝色方
            if set_team_color == 1:  
                if Treasure == "BlueTrue":  #* 如果是蓝色真宝藏
                    TreasureIfo[NextTreasure][4] = 2
                    TreasureIfo[NextTreasure][6] = 1
                    TreasureIfo[NextTreasure][5] = 0
                    # 同一个象限的另一个宝藏一定为红色
                    for i in range(8): 
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][5] = 0
                    # 处于镜像对称点的宝藏一定为红色真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][6] = 1
                            TreasureIfo[i][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 2
                
                elif Treasure == "BlueFalse":  #* 如果是蓝色假宝藏
                    TreasureIfo[NextTreasure][4] = 2
                    TreasureIfo[NextTreasure][6] = 2
                    TreasureIfo[NextTreasure][5] = 0
                    # 除了镜像对称的宝藏都是真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] != TreasureIfo[NextTreasure][3]:
                            TreasureIfo[i][6] = 1
                    # 同一个象限的另一个宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][5] = 0
                    # 处于镜像对称点的宝藏一定为红色假宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][6] = 2
                            TreasureIfo[i][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 2

                elif Treasure == "RedTrue": #* 如果是红色真宝藏
                    TreasureIfo[NextTreasure][4] = 1
                    TreasureIfo[NextTreasure][6] = 1
                    TreasureIfo[NextTreasure][5] = 0
                    # 同一个象限的另一个宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 2
                    # 处于镜像对称点的宝藏一定为蓝色真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][6] = 1
                    # 和镜像对称点在同一个象限的宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][5] = 0

                elif Treasure == "RedFalse": # *如果是红色假宝藏
                    TreasureIfo[NextTreasure][4] = 1
                    TreasureIfo[NextTreasure][6] = 2
                    TreasureIfo[NextTreasure][5] = 0
                    # 除了镜像对称的宝藏都是真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] != TreasureIfo[NextTreasure][3]:
                            TreasureIfo[i][6] = 1
                    # 同一个象限的另一个宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 2
                    # 处于镜像对称点的宝藏一定为蓝色假宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][6] = 2
                            TreasureIfo[i][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][5] = 0
                else:# *如果是无宝藏
                    print("真的没看到宝藏哇")
                    TreasureIfo[NextTreasure][5] = 0 # 将宝藏点标记为未激活

            #! 如果是红色方
            if set_team_color == 0:  
                if Treasure == "BlueTrue":  #* 如果是蓝色真宝藏
                    TreasureIfo[NextTreasure][4] = 2
                    TreasureIfo[NextTreasure][6] = 1
                    TreasureIfo[NextTreasure][5] = 0
                    # 同一个象限的另一个宝藏一定为红色
                    for i in range(8): 
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 1
                    # 处于镜像对称点的宝藏一定为红色真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][6] = 1
                    # 和镜像对称点在同一个象限的宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][5] = 0

                elif Treasure == "BlueFalse":  #* 如果是蓝色假宝藏
                    TreasureIfo[NextTreasure][4] = 2
                    TreasureIfo[NextTreasure][6] = 2
                    TreasureIfo[NextTreasure][5] = 0
                    # 除了镜像对称的宝藏都是真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] != TreasureIfo[NextTreasure][3]:
                            TreasureIfo[i][6] = 1
                    # 同一个象限的另一个宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 1
                    # 处于镜像对称点的宝藏一定为红色假宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 1
                            TreasureIfo[i][6] = 2
                            TreasureIfo[i][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][5] = 0
                elif Treasure == "RedTrue": #* 如果是红色真宝藏
                    TreasureIfo[NextTreasure][4] = 1
                    TreasureIfo[NextTreasure][6] = 1
                    TreasureIfo[NextTreasure][5] = 0
                    # 同一个象限的另一个宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][5] = 0
                    # 处于镜像对称点的宝藏一定为蓝色真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[mirror][4] = 2
                            TreasureIfo[mirror][6] = 1
                            TreasureIfo[mirror][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 1
                elif Treasure == "RedFalse": # *如果是红色假宝藏
                    TreasureIfo[NextTreasure][4] = 1
                    TreasureIfo[NextTreasure][6] = 2
                    TreasureIfo[NextTreasure][5] = 0
                    # 除了镜像对称的宝藏都是真宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] != TreasureIfo[NextTreasure][3]:
                            TreasureIfo[i][6] = 1
                    # 同一个象限的另一个宝藏一定为蓝色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[NextTreasure][2] and i != NextTreasure:
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][5] = 0
                    # 处于镜像对称点的宝藏一定为蓝色假宝藏
                    for i in range(8):
                        if TreasureIfo[i][3] == TreasureIfo[NextTreasure][3] and i != NextTreasure:
                            mirror = i
                            TreasureIfo[i][4] = 2
                            TreasureIfo[i][6] = 2
                            TreasureIfo[i][5] = 0
                    # 和镜像对称点在同一个象限的宝藏一定为红色
                    for i in range(8):
                        if TreasureIfo[i][2] == TreasureIfo[mirror][2] and i != mirror:
                            TreasureIfo[i][4] = 1
                else:# *如果是无宝藏
                    print("真的没看到宝藏哇")
                    TreasureIfo[NextTreasure][5] = 0 # 将宝藏点标记为未激活

            # TODO 展示当前已知的宝藏信息
            print("当前已知宝藏信息为:", TreasureIfo)
            # 将已知宝藏的颜色信息显示在地图上
            for i in range(8):
                if TreasureIfo[i][4] == 1: # 红色宝藏  
                    if TreasureIfo[i][6] == 1: # 红色真宝藏
                        cv2.circle(cropimg,(cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (255, 0, 0), -1)
                        cv2.circle(cropimg,(cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 15, (0, 255, 0), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是红色真宝藏")
                    elif TreasureIfo[i][6] == 2: # 红色假宝藏
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (255, 0, 0), -1)
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 15, (255, 255, 0), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是红色假宝藏")
                    else:# 红色未知宝藏
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (255, 0, 0), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是红色未知宝藏")

                
                elif TreasureIfo[i][4] == 2: # 蓝色宝藏 
                    if TreasureIfo[i][6] == 1: # 蓝色真宝藏
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (0, 0, 255), -1)
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 15, (255, 255, 0), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是蓝色真宝藏")
                    elif TreasureIfo[i][6] == 2: # 蓝色假宝藏
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (0, 0, 255), -1)
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 15, (0, 255, 0), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是蓝色假宝藏")
                    else: # 蓝色未知宝藏
                        cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (0, 0, 255), -1)
                        print(TreasureIfo[i][0], TreasureIfo[i][1],"是蓝色未知宝藏")
                
                else: # 未知宝藏
                    # cv2.circle(cropimg, (cell * TreasureIfo[i][0] + int(cell * 3), cell * TreasureIfo[i][1] + int(cell * 1)), 25, (0, 0, 0), -1)
                    print(TreasureIfo[i][0], TreasureIfo[i][1],"是未知宝藏")

            qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
                                QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            # TODO 发送撞击指令与否
            # 如果是蓝色方，且为真宝藏，发送撞击指令
            if set_team_color == 1:
                if Treasure == "BlueTrue":
                    ifhit = 1
                    True_Treas_Num += 1
                else:
                    ifhit = 0

            # 如果是红色方，且为真宝藏，发送撞击指令
            else:
                if Treasure == "RedTrue":
                    ifhit = 1
                    True_Treas_Num += 1
                else:
                    ifhit = 0
            print("ifhit:", ifhit)
            # step_next, none, length_next = CreateStep(itel + 1)
            # TODO 发送撞击指令
            # time.sleep(0.5)
            if ifhit == 1:
                while True:
                    serialapi.communicate(0x00, 0x01, [[0x00]], hitdirection, eval(hex(hitdistance)))
                    time.sleep(0.05)
                    #!!!!因调试注释掉：
                    time.sleep(1)
                    # if str(serialapi.recv) == '000100ff':
                    break# 等待接收到回复信号
                    if ShouldStop == 1: break; # 如果停止了，结束当前进程
                serialapi.recv = str.encode('xxxxxxxxxxx')
                if ShouldStop == 1: break; # 如果停止了，结束当前进程
            else:
                print ("不打不打")
            
            # TODO 计算下一次应该去哪个宝藏点
            # 通过调用距离矩阵，找到最近的仍然激活的宝藏点
            Temp_Next_Treasure = 999
            Temp_Distance = 999
            for i in range(8):
                if Trea_distances[NextTreasure][i] < Temp_Distance and TreasureIfo[i][5] == 1: # 如果距离比上一个小,且未激活,则更新
                    print("Trea_distances[NextTreasure][i]:", Trea_distances[NextTreasure][i])
                    Temp_Next_Treasure = i # 更新下一个宝藏点
                    Temp_Distance = Trea_distances[NextTreasure][i] # 更新距离
            
            if Temp_Next_Treasure == 999: # 如果没有找到下一个宝藏点，则说明所有宝藏点都已经失活
                CurrentTreasure = NextTreasure # 更新当前宝藏点
                print("所有宝藏点都已经失活")
            else:
                CurrentTreasure = NextTreasure # 更新当前宝藏点
                NextTreasure = Temp_Next_Treasure # 更新下一个宝藏点
                print("下一个宝藏点的坐标为:", treasureinmap[NextTreasure][0], treasureinmap[NextTreasure][1]) # # 并打印下一个宝藏点的坐标


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
