#-*-coding:utf-8-*-
# Function: Main program
#? 主程序
#TODO Version 0.2.20230715
#! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py | networkx | itertools
#* Thread 利用情况：Thread-0 UART通信
#* Thread 利用情况：Thread-1 路径规划线程
#* Thread 利用情况：Thread-2 地图扫描线程
import sys,cv2,time,threading # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton # 导入PyQt5模块
from PyQt5 import QtGui,QtCore # 导入PyQt5GUI模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours # 导入地图寻路部分
from MapScan import  reshape_image_scan, detect, find,affine_transformation # 导入地图扫描部分
from Astar import Map,MapNode,astar, tsp # 导入A*算法部分
import Identify # 导入识别模块
import numpy as np # 导入Numpy模块
import serialapi # 导入串口模块

import networkx as nx
import itertools

import faulthandler
faulthandler.enable()

CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 720 # 设定的相机取样像素参数
result_final = [] # 寻路结果存储
My_True_Num = 0 # 已遍历真实宝藏点数目

def CreateStep(itel, final, false): #* 创建步进函数
    global result_final, boardmap, My_True_Num, treasureinmap, hitdirection, hitdistance
    # TODO 计算此路段的行进方式

    if (false == 1):  # 如果false为1，则进入不碰撞快速通过模式
        # 设置一个新的坐标，将宝藏点坐标存入其中
        address = treasureinmap[itel - 1]
        # 设置当前坐标为宝藏点坐标减去最后一步的方向和距离
        if (hitdirection == 0x00):
            address = [treasureinmap[itel - 1][0] - hitdistance * 2, treasureinmap[itel - 1][1]]
        elif (hitdirection == 0x01):
            address = [treasureinmap[itel - 1][0], treasureinmap[itel - 1][1] + hitdistance * 2]
        elif (hitdirection == 0x02):
            address = [treasureinmap[itel - 1][0] + hitdistance * 2, treasureinmap[itel - 1][1]]
        elif (hitdirection == 0x03):
            address = [treasureinmap[itel - 1][0], treasureinmap[itel - 1][1] - hitdistance * 2]
        print("address", address)
        if final == 1:
            map = Map(boardmap, address[1], address[0], 0, 18, )
            print("next_destination:", 0, 18)
        else:
            map = Map(boardmap, address[1], address[0], treasureinmap[itel][1], treasureinmap[itel][0], )
            print("next_destination:", treasureinmap[itel][0], treasureinmap[itel][1])
        result_this_time = astar(map)
        result_this_time.reverse()
    # 提取result_final[itel]中的点，存到一个列表result_this_time中
    else:
        if final == 0:
            map = Map(boardmap, treasureinmap[itel][1], treasureinmap[itel][0], 0, 18, )
            print("trea", treasureinmap[itel][1], treasureinmap[itel][0])
            result_this_time = astar(map)
            result_this_time.reverse()
        else:
            result_this_time = result_final[itel]
    # step为一个30x2的二维数组，每一行代表一个数据帧，第一列代表功能码，第二列代表数据。
    # 功能码为0x00代表向北走，0x01代表向东走，0x02代表向南走，0x03代表向西走。
    # 北在地图右方，东在地图下方，南在地图左方，西在地图上方。

    # treasureinmap[itel][0]为y坐标，treasureinmap[itel][1]为x坐标

    #todo                  0x01↑
    #todo             0x02←       0x00→
    #todo                  0x03↓


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
    hitdirection = step_new[-1][0] # 方向
    hitdistance = step_new[-1][1] # 距离

    
    # 将step_new最后一步减去
    # 如果这是最后一段路程，则步数加1
    if (itel == len(result_final) - 1):
        step_new[-1][1] += 1
    else:
        # 将step_new[-1]从step_new中删除
        step_new.pop(-1)
    for i in range(len(step_new)):
        if (step_new[i][0] == 0x00):
            print ("→", step_new[i][1])
        elif (step_new[i][0] == 0x03):
            print ("↓", step_new[i][1])
        elif (step_new[i][0] == 0x02):
            print ("←", step_new[i][1])
        elif (step_new[i][0] == 0x01):
            print ("↑", step_new[i][1])
    # 创建一个2xlength的矩阵step_final，将step中的数据存入其中
    length = len(step_new)
    step_final = np.zeros((2, length), dtype=np.uint8)
    for i in range(length):
        step_final[0][i] = step_new[i][0]
        step_final[1][i] = step_new[i][1]
    # 把step_final转换成1 x length * 2的数组
    step_final = step_final.reshape(1, length * 2)
    print("step_final", step_final)
    return step_final, length * 2
'''  类封装  '''
class Camera: #TODO 相机调取类封装



    def __init__(self,camera):
        self.frame = []
        self.ret = False
        self.cap = 0
        self.camera = camera
    def open(self):
        self.cap = cv2.VideoCapture(self.camera)
        self.ret = self.cap.set(3, CAMERA_WIDTH)
        self.ret = self.cap.set(4, CAMERA_HEIGHT)
        self.cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
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

class Example(QWidget): #TODO 主窗口类


    def __init__(self): #* 初始化函数
        super().__init__()

        self.initUI()


    def initUI(self): #* 界面初始化函数

        self.lbl = QLabel('系统初始化进行中', self)
        self.lbl.move(0, 10)
        self.lbl.setFixedSize(600, 30)
        self.lbl.setAlignment(QtCore.Qt.AlignCenter) # 修改字体居中
        self.lbl.setFont(QtGui.QFont("Arial", 14)) # 修改字体大小为16px

        self.sublbl = QLabel(' 等待路径规划完成后运行', self)
        self.sublbl.move(0, 40)
        self.sublbl.setFixedSize(600, 30)
        self.sublbl.setAlignment(QtCore.Qt.AlignCenter) # 修改字体居中
        self.sublbl.setFont(QtGui.QFont("Arial", 10)) # 修改字体大小为12px

        # 拍照按钮
        self.stbtn = QPushButton('拍摄藏宝图', self)
        self.stbtn.clicked.connect(self.startup_thread) # 修改按钮行为为启动特定函数“Startup”
        self.stbtn.move(40, 80)
        self.stbtn.setFixedSize(160, 50)
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
        self.runbtn.clicked.connect(self.runcar_thread) # 修改按钮行为为启动小车运行函数
        self.runbtn.move(220, 80)
        self.runbtn.setFixedSize(180, 50)
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
        self.runbtn.setEnabled(False) # 禁用按钮

        # 选择红色方
        self.redbtn = QPushButton('红方', self)
        self.redbtn.clicked.connect(self.red_thread) # 修改按钮行为为启动红方线程
        self.redbtn.move(15, 220)
        self.redbtn.setFixedSize(100, 100)
        # 设置按钮样式
        self.redbtn.setStyleSheet("QPushButton:hover{color:black}" # 光标移动到上面后的前景色
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
                                  "QPushButton{color:black}" # 按键前景色
                                  "QPushButton{background-color:rgb(190,0,0)}"  # 按键背景色
                                  "QPushButton{border:0px}"  # 边界宽度
                                  "QPushButton{border-radius:20px}"  # 边界圆角半径
                                  "QPushButton{padding:2px 4px}"  # 按键内边界
                                  "QPushButton{font-family:'黑体';font-size:35px}")  # 字体样式
        # 选择蓝色方
        self.bluebtn = QPushButton('蓝方', self)
        self.bluebtn.clicked.connect(self.blue_thread)
        self.bluebtn.move(485, 220)
        self.bluebtn.setFixedSize(100, 100)
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
        self.btn = QPushButton('退出程序', self)
        # self.btn.clicked.connect(self.quit)
        self.btn.move(420, 80)
        self.btn.setFixedSize(140, 50)
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
        self.setGeometry(340, 240, 600, 400)
        self.setWindowTitle('宝藏程序')
        self.setFont(QtGui.QFont("Arial", 16)) # 修改字体大小为16px

        # 添加一个用于显示相机图像的控件
        self.camera_label = QLabel(self)
        self.camera_label.move(130, 150)
        self.camera_label.setFixedSize(340, 240)
        self.camera_label.setScaledContents(True)
        # 加载一张图片
        pixmap = QtGui.QPixmap('test.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)
        self.show()


        # 系统串口检测
        self.lbl.setText('检测串口通信通路')
        if serialapi.If_Serial_Open == False: # 检测串口是否打开
            self.lbl.setText('串口通信通路异常,系统初始化失败')
        else:
            threading.Thread(target=serialapi.uartRx, args=()).start() #* Thread-0 开启串口接收子线程
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
        

    def red_thread(self): #* 启动红方线程
        t = threading.Thread(target=self.red)
        t.start()


    def blue_thread(self): #* 启动蓝方线程
        t = threading.Thread(target=self.blue)
        t.start()


    def startup_thread(self): #* 启动路径规划线程
        t = threading.Thread(target=self.Startup)
        t.start()


    def runcar_thread(self): #* 启动运行线程
        t = threading.Thread(target=self.Runcar)
        t.start()


    def red(self): #* 红方线程
        global set_team_color
        set_team_color = 0
        self.redbtn.setEnabled(False)
        self.bluebtn.setEnabled(False)
        self.stbtn.setEnabled(True)
        print("set_team_color:", set_team_color)


    def blue(self): #* 蓝方线程
        global set_team_color
        set_team_color = 1
        self.redbtn.setEnabled(False)
        self.bluebtn.setEnabled(False)
        self.stbtn.setEnabled(True)
        print("set_team_color:", set_team_color)


    def Startup(self): #* 启动函数
        global result_final, boardmap, map_gui_image, cell, map_cv2_image, treasureinmap
        sys.setrecursionlimit(10000)
        global result_final,boardmap
        # camera = Camera(1) # 打开相机
        # camera.open()
        # 等待调整相机并拍照
        last_image = None
        # start_time = time.time()
        # scan_time = 3
        # while time.time() - start_time < scan_time:
        #     ret, image = camera.read() # 读取相机图像
        #     # 将相机图像显示在控件中
        #     qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0],image.shape[1]*3, QtGui.QImage.Format_RGB888)
        #     pixmap = QtGui.QPixmap.fromImage(qimg)
        #     self.camera_label.setPixmap(pixmap)
        #     self.lbl.setText('调节时间剩余'+str(int(scan_time-time.time() + start_time))+"秒，请注意")
        #     last_image = image
        #
        # camera.release()  # 释放相机资源1
        last_image = cv2.imread('./test/mapwithpoint1.jpg')  # 读取test.jpg

        #? 照片扫描加纠偏部分开始
        start_time = time.time()
        self.lbl.setText('照片扫描纠偏中...')
        image, new_width, new_height = reshape_image_scan(last_image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image, contours, hierachy = detect(image)
        rec ,img= find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0],img.shape[1]*3, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 照片扫描加纠偏部分结束

        #? 地图扫描输出数组部分开始
        self.lbl.setText('地图扫描输出进行中...')
        img, width, height = reshape_image_find(img)  # 调整图片大小
        ToBinray(img)  # 转二进制
        contours, boardx, cropimg, cell, treasureinmap = GetGontours(img)  # 提取轮廓
        for i in range(len(boardx)):  # 将board中255的值转换为1
            for j in range(len(boardx[0])):
                if(boardx[i][j]==255):
                    boardx[i][j]=1
        boardmap = tuple(boardx)  # 将board转换为元组
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 地图扫描输出数组部分结束

        self.lbl.setText('路径规划进行中...')
        #? 路径规划部分开始
        treasureinmap = tsp(boardmap, treasureinmap)  # 调用tsp算法
        #? 路径规划部分结束
        end_time = time.time()
        print("time:",end_time - start_time)
        result_final = []
        for j in range(-1,8,1):
            if j==-1: map = Map(boardmap, 18,0,treasureinmap[j+1][1],treasureinmap[j+1][0],)
            elif j==7: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],0,18,)
            else: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],treasureinmap[j+1][1],treasureinmap[j+1][0],)  # (mapdata,startx,starty,endx,endy)
            result = astar(map)
            result.reverse()
            resultx = np.asarray(result)
            # print("len",len(resultx))
            for i in range(len(resultx)): # 绘制线段
                if i != 0:
                    cv2.line(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)),
                            (cell * resultx[i - 1][1] + int(cell * 3), cell * resultx[i - 1][0] + int(cell * 1)), (0, 0, 255), 2)
                cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2, (255, 0, 0), 2)
            result_final.append(result)
        print("怎么移动：", result_final)
        print(result_final[0][0][0],result_final[0][0][1])
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # 界面更新
        self.stbtn.setEnabled(False)
        self.runbtn.setEnabled(True)
        self.lbl.setText('路径规划完成,请按下Enter运行...')





    def Runcar(self): #* 运行函数
        global result_final, boardmap,  treasureinmap, ifhit, hitdistance, hitdirection, My_True_Num, My_False_Num, Your_True_Num, Your_False_Num
        # 数据初始化
        My_True_Num = 0
        ifhit = 1


        #TODO 运行8段路程，完成宝藏遍历
        for itel in range(len(result_final)):
            self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点')
            # # TODO 计算此路段的行进方式
            # # 提取result_final[itel]中的点，存到一个列表result_this_time中
            # result_this_time = result_final[itel]
            # # step为一个30x2的二维数组，每一行代表一个数据帧，第一列代表功能码，第二列代表数据。
            # # 功能码为0x00代表向北走，0x01代表向东走，0x02代表向南走，0x03代表向西走。
            # # 北在地图右方，东在地图下方，南在地图左方，西在地图上方。
            # # 数据代表前进的步数，为0x00~0x64，即0~100。

            # 测试用，True_Treas_Num = 3
            # True_Treas_Num = 3

            if ifhit == 1: # 如果上次击打了
                # TODO 计算地图
                if My_True_Num == 3: # 如果已经找到3个宝藏，那么就计算从当前位置到终点的路径
                    step_final, length = CreateStep(itel, 1, 0)
                else: # 否则计算从当前位置到下一个宝藏的路径
                    step_final, length = CreateStep(itel, 0, 0)



                # TODO  使用serialapi.communicate将step发给下位机
                while True:
                    if str(serialapi.recv) == '000100ff': break
                    serialapi.communicate(0x01, eval(hex(length)), step_final, hitdirection, eval(hex(hitdistance)))
                    time.sleep(0.1)
                serialapi.recv = str.encode('xxxxxxxxxxx')
                if My_True_Num == 3:
                    self.lbl.setText('运行中...前往终点')
                    self.sublbl.setText("前往终点")
                    print("前往终点")
                else:
                    self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点')
                    self.sublbl.setText("前往宝藏点"+str(itel+1))
                    print("前往宝藏点",str(itel+1))

            #TODO 等待运行完成
            while True:
                if str(serialapi.recv) == '000000ff': break;  # 等待接收到回复信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            print("第"+str(itel+1)+"个宝藏点已到达")
            self.sublbl.setText("第"+str(itel+1)+"个宝藏点已到达")

            # 如果到达终点则结束
            if itel == len(result_final) - 1 or My_True_Num == 3:
                self.lbl.setText('运行结束')
                self.sublbl.setText('运行结束')
                break
            time.sleep(1)
            # TODO 拍照识别
            # camera = Camera(1) # 打开相机
            # camera.open()
            # start_time = time.time()
            # while time.time() - start_time < 2:
            #     ret, Treas_image = camera.read() # 读取相机宝藏图像
            #     Treas_qimg = QtGui.QImage(Treas_image.data, Treas_image.shape[1], Treas_image.shape[0],Treas_image.shape[1]*3, QtGui.QImage.Format_RGB888)
            #     Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
            #     self.camera_label.setPixmap(Treas_pixmap)
            #     self.sublbl.setText('剩余'+str(int(2-time.time() + start_time))+"秒进行拍摄识别")
            # camera.release()  # 释放相机资源
            # Treas_img_copy = Treas_image.copy()
            # Treas_image = reshape_image_scan(Treas_image)[0]
            # copy_img = Treas_image

            # Treas_image, contours, yellow = Identify.FindColorOne(Treas_img_copy, 1)  # 黄色
            # Treas_image, contours, green = Identify.FindColorOne(Treas_img_copy, 2)  # 绿色
            # Treas_image, contours, blue = Identify.FindColorOne(Treas_img_copy, 0)  # 蓝色
            # Treas_image, contours, red = Identify.FindRedOne(Treas_img_copy, contours)  # 红色
            # #蓝色1，黄色1，为蓝色真宝藏；红色1，绿色1，为红色真宝藏；蓝色1，绿色1，为蓝色假宝藏；红色1，黄色1，为红色假宝藏

            # 走流程识别，固定为红色真宝藏
            blue = 0
            yellow = 0
            red = 1
            green = 1

            if blue == 1 and yellow == 1:
                self.sublbl.setText('蓝色真宝藏');print("蓝色真宝藏");Treasure = "BlueTrue"
            elif red == 1 and green == 1:
                self.sublbl.setText('红色真宝藏');print("红色真宝藏");Treasure = "RedTrue"
            elif blue == 1 and green == 1:
                self.sublbl.setText('蓝色假宝藏');print("蓝色假宝藏");Treasure = "BlueFalse"
            elif red == 1 and yellow == 1:
                self.sublbl.setText('红色假宝藏');print("红色假宝藏");Treasure = "RedFalse"
            else:
                self.sublbl.setText('无宝藏');print("无宝藏");Treasure = "None"


            # Identify.ShapeDetection(copy_img, contours, Treasure)  #形状检测
            # Treas_qimg = QtGui.QImage(copy_img.data, copy_img.shape[1], copy_img.shape[0],copy_img.shape[1]*3, QtGui.QImage.Format_RGB888)
            # Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
            # self.camera_label.setPixmap(Treas_pixmap)
            time.sleep(0.1)

            # TODO 发送撞击指令与否
            # 如果是蓝色方
            if set_team_color == 1 :
                if Treasure == "BlueTrue":
                    ifhit = 1
                    My_True_Num += 1
                elif Treasure == "BlueFalse":
                    ifhit = 0
                    My_False_Num += 1
                elif Treasure == "RedTrue":
                    ifhit = 0
                    Your_True_Num += 1
                elif Treasure == "RedFalse":
                    ifhit = 0
                    Your_False_Num += 1
                else:
                    ifhit = 0

            # 如果是红色方，且为真宝藏，发送撞击指令
            elif set_team_color == 0:
                if Treasure == "RedTrue":
                    ifhit = 1
                    My_True_Num += 1
                elif Treasure == "RedFalse":
                    ifhit = 0
                    My_False_Num += 1
                elif Treasure == "BlueTrue":
                    ifhit = 0
                    Your_True_Num += 1
                elif Treasure == "BlueFalse":
                    ifhit = 0
                    Your_False_Num += 1
                else:
                    ifhit = 0

            print("ifhit:",ifhit)
            print("My_True_Num",My_True_Num)
            # step_next, none, length_next = CreateStep(itel + 1)
              # TODO 发送撞击指令
            time.sleep(0.5)
            # 如果是真宝藏，发送撞击指令
            if ifhit == 1:
                while True:
                    if str(serialapi.recv) == '000100ff': break
                    serialapi.communicate(0x00, 0x01, [[0x00]], ifhit, eval(hex(hitdistance)))
                    time.sleep(0.1)
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.lbl.setText("撞击中")
                self.sublbl.setText("撞击中")
            
            # 如果是假宝藏，前往下一个宝藏
            else:    
                # 如果不是最后一段路程，发送下一段路程:
                print("itel:", itel)
                print("len(result_final):", len(result_final))
                if itel != len(result_final) - 2:

                    step_final, length = CreateStep(itel + 1, 0, 1)
                    while True:
                        if str(serialapi.recv) == '000100ff': break
                        serialapi.communicate(0x01, eval(hex(length)), step_final, hitdirection, eval(hex(hitdistance)))
                        time.sleep(0.1)
                    serialapi.recv = str.encode('xxxxxxxxxxx')
                    self.lbl.setText('运行中...前往第' + str(itel + 2) + '个宝藏点')
                    self.sublbl.setText("没打击")
                # 如果是最后一段路程，发送去终点的路程:
                else:
                    step_final, length = CreateStep(itel + 1, 1, 1)
                    while True:
                        if str(serialapi.recv) == '000100ff': break
                        serialapi.communicate(0x01, eval(hex(length)), step_final, hitdirection, eval(hex(hitdistance)))
                        time.sleep(0.1)
                    serialapi.recv = str.encode('xxxxxxxxxxx')
                    self.lbl.setText('运行中...前往最后的出口')
                    self.sublbl.setText("没打击")
                    print("最后一段咯")

                
            


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
