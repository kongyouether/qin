# -*-coding:utf-8-*-
# Function: Main program
# ? 主程序
# TODO Version 2.0.20230803
# ! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py | Identify.py | serialapi.py | networkx | itertools
# * Thread 利用情况：Thread-0 UART通信
# * Thread 利用情况：Thread-1 路径规划线程
# * Thread 利用情况：Thread-2 地图扫描线程
import sys, cv2, time, threading  # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QInputDialog  # 导入PyQt5模块
from PyQt5 import QtGui, QtCore  # 导入PyQt5GUI模块
from PyQt5.QtCore import QUrl  # 导入PyQt5多媒体模块
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent  # 导入PyQt5多媒体模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours  # 导入地图寻路部分
from MapScan import reshape_image_scan, detect, find, affine_transformation  # 导入地图扫描部分
from Astar_cy import Map, astar, tsp  # 导入A*算法部分
import Identify_cy  # 导入识别模块
import numpy as np  # 导入Numpy模块
import serialapi  # 导入串口模块

# import networkx as nx # 导入网络图模块
# import itertools # 导入迭代器模块

CAMERA_WIDTH = 1280;
CAMERA_HEIGHT = 720  # 设定的相机取样像素参数
result_final = []  # 寻路结果存储
car_color_group = 'RED'  # 小车颜色存储，默认为红色
map_gui_image = None;
map_cv2_image = None  # 地图路径规划完成图像存储
cell = None  # 地图单元格大小
True_Treas_Num = 0  # 已遍历真实宝藏点数目
Treas_distances = [];
treasureinmap_ori = []  # 宝藏点之间存储

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
        global car_color_group  # 全局变量调用
        self.lbl = QLabel('系统初始化进行中', self)
        self.lbl.move(0, 10)
        self.lbl.setFixedSize(600, 30)
        self.lbl.setAlignment(QtCore.Qt.AlignCenter)  # 修改字体居中
        self.lbl.setFont(QtGui.QFont("Arial", 16))  # 修改字体大小为16px

        self.sublbl = QLabel(' 等待路径规划完成后运行', self)
        self.sublbl.move(0, 40)
        self.sublbl.setFixedSize(600, 30)
        self.sublbl.setAlignment(QtCore.Qt.AlignCenter)  # 修改字体居中
        self.sublbl.setFont(QtGui.QFont("Arial", 12))  # 修改字体大小为12px

        # 启动按钮
        self.stbtn = QPushButton('拍摄藏宝图', self)
        self.stbtn.clicked.connect(self.startup_thread)  # 修改按钮行为为启动特定函数“Startup”
        self.stbtn.move(40, 80)
        self.stbtn.setFixedSize(160, 50)
        # 仿真按钮
        self.simbtn = QPushButton(self)
        self.simbtn.setGeometry(40, 150, 160, 50)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(simunready.png);}")
        self.simbtn.setFixedSize(72, 72)
        self.simbtn.clicked.connect(self.simulate_thread)  # 修改按钮行为为启动特定函数“Simulate”
        self.simbtn.setEnabled(False)
        # 运行按钮
        self.runbtn = QPushButton('启动AutoCar运行', self)
        self.runbtn.clicked.connect(self.runcar_thread)  # 修改按钮行为为启动小车运行函数
        self.runbtn.move(220, 80)
        self.runbtn.setFixedSize(180, 50)
        self.runbtn.setEnabled(False)
        # 退出按钮
        self.btn = QPushButton('退出程序', self)
        self.btn.clicked.connect(QApplication.instance().quit)
        self.btn.move(420, 80)
        self.btn.setFixedSize(140, 50)

        self.setGeometry(100, 40, 600, 400)
        self.setWindowTitle('CARSYS')
        self.setWindowIcon(QtGui.QIcon('icon.png'))
        self.setFont(QtGui.QFont("Arial", 16))  # 修改字体大小为16px

        # 创建QMediaPlayer对象
        self.bgmPlayer = QMediaPlayer()
        # 加载音乐文件
        MusicUrl = QUrl.fromLocalFile("/home/rock/Desktop/PM06Master/bgm.mp3")
        MusicContent = QMediaContent(MusicUrl)
        self.bgmPlayer.setMedia(MusicContent)

        # 添加一个用于显示相机图像的控件
        self.camera_label = QLabel(self)
        self.camera_label.move(130, 150)
        self.camera_label.setFixedSize(340, 240)
        self.camera_label.setScaledContents(True)
        # 加载一张图片
        pixmap = QtGui.QPixmap('layla.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)
        self.show()

        # 系统串口检测
        self.lbl.setText('检测串口通信通路')
        if serialapi.If_Serial_Open == False:  # 检测串口是否打开
            self.lbl.setText('串口通信通路异常,系统初始化失败')
        else:
            threading.Thread(target=serialapi.uartRx, args=()).start()  # * Thread-0 开启串口接收子线程
            serialapi.communicate(0xaa, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x00)  # 发送启动信号
            start_time = time.time();
            Serial_response = 0
            while time.time() - start_time < 8:
                if str(serialapi.recv)[0:14] == 'aa01a100000000':
                    serialapi.recv = str.encode('xxxxxxxxxxx')
                    Serial_response = 1
                    self.lbl.setText('串口通信通路正常,系统初始化完成')
                    break;  # 等待接收到启动信号
            if Serial_response == 0: self.lbl.setText('串口通信通路异常,系统初始化失败')

        # 选择阵营
        car_color_group, ok = QInputDialog.getItem(None, '选择颜色', '请选择当前是红方还是蓝方：', ['RED', 'BLUE'])
        self.sublbl.setText('当前阵营为：' + car_color_group + ",等待路径规划完成后运行")

    def startup_thread(self):  # * 启动路径规划线程
        t = threading.Thread(target=self.Startup)
        t.start()

    def simulate_thread(self):  # * 启动仿真线程
        t = threading.Thread(target=self.Simulate)
        t.start()

    def runcar_thread(self):  # * 启动运行线程
        t = threading.Thread(target=self.Runcar)
        t.start()

    def Startup(self):  # * 启动函数
        global result_final, boardmap, map_gui_image, cell, map_cv2_image, treasureinmap, Treas_distances, treasureinmap_ori
        camera = Camera(0)  # 打开相机
        camera.open()
        # 等待调整相机并拍照
        last_image = None
        start_time = time.time()
        while time.time() - start_time < 5:
            ret, image = camera.read()  # 读取相机图像
            # 将相机图像显示在控件中
            qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0], image.shape[1] * 3,
                                QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.lbl.setText('藏宝图调节时间剩余' + str(int(5 - time.time() + start_time)) + "秒，请注意")
            last_image = image

        camera.release()  # 释放相机资源
        last_image = cv2.imread('./test/mapwithpoint1.jpg')  # 读取test.jpg

        # ? 照片扫描加纠偏部分开始
        time_calc = time.time()
        self.lbl.setText('照片扫描纠偏中...')
        image, new_width, new_height = reshape_image_scan(last_image)
        image, contours, hierachy = detect(image)
        rec, img = find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * 3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # ? 照片扫描加纠偏部分结束

        # ? 地图扫描输出数组部分开始
        self.lbl.setText('地图扫描输出进行中...')
        img, width, height = reshape_image_find(img)  # 调整图片大小
        ToBinray(img)  # 转二进制
        contours, boardx, cropimg, cell, treasureinmap = GetGontours(img)  # 提取轮廓
        for i in range(len(boardx)):  # 将board中255的值转换为1
            for j in range(len(boardx[0])):
                if (boardx[i][j] == 255): boardx[i][j] = 1
        boardmap = tuple(boardx)  # 将board转换为元组
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
                            QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # ? 地图扫描输出数组部分结束

        self.lbl.setText('路径规划进行中...')
        treasureinmap_ori = treasureinmap  # 保存原始的宝藏坐标
        treasureinmap, Treas_distances = tsp(boardmap, treasureinmap)  # 调用tsp算法
        result_final = []
        for j in range(-1, 8, 1):
            if j == -1:
                map = Map(boardmap, 18, 0, treasureinmap[j + 1][1], treasureinmap[j + 1][0], )
            elif j == 7:
                map = Map(boardmap, treasureinmap[j][1], treasureinmap[j][0], 0, 18, )
            else:
                map = Map(boardmap, treasureinmap[j][1], treasureinmap[j][0], treasureinmap[j + 1][1],
                          treasureinmap[j + 1][0], )  # (mapdata,startx,starty,endx,endy)
            result = astar(map)  # 调用astar算法
            result.reverse()
            resultx = np.asarray(result)
            for i in range(len(resultx)):  # 绘制线段
                if i != 0:
                    cv2.line(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)),
                             (cell * resultx[i - 1][1] + int(cell * 3), cell * resultx[i - 1][0] + int(cell * 1)),
                             (0, 0, 255), 2)
                cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2,
                           (255, 0, 0), 2)
            result_final.append(result)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3,
                            QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        map_gui_image = pixmap
        map_cv2_image = cropimg
        self.camera_label.setPixmap(pixmap)
        # 界面更新
        self.stbtn.setEnabled(False)
        self.runbtn.setEnabled(True)
        self.simbtn.setEnabled(True)
        self.lbl.setText(
            '路径规划完成,耗时{:.2f}s'.format(time.time() - time_calc) + ',可以进行路径模拟或按下Enter运行...')
        self.simbtn.setStyleSheet("QPushButton { border-image: url(sim.png);}")

    def Simulate(self):  # * 仿真函数
        global result_final, boardmap, cell, map_cv2_image
        self.simbtn.setEnabled(False)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(simunready.png);}")  # 按钮状态修改为不可执行，等待模拟完成后解除
        self.lbl.setText('模拟系统启动，正在模拟运行...')
        sim_timer = time.time()  # 计时器开始
        for index, router in enumerate(result_final):
            temp_img = map_cv2_image.copy()
            self.sublbl.setText('正在前往目标点' + str(index + 1) + '...')
            for j in range(len(router)):  # 绘制线段
                cv2.circle(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)), 2,
                           (0, 0, 255), 6)
                if j != 0:
                    cv2.line(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)),
                             (cell * router[j - 1][1] + int(cell * 3), cell * router[j - 1][0] + int(cell * 1)),
                             (255, 0, 0), 6)
                cv2.circle(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)), 2,
                           (0, 0, 255), 6)
                if (j % 3 == 1):
                    cv2.circle(temp_img, (cell * router[-1][1] + int(cell * 3), cell * router[-1][0] + int(cell * 1)),
                               2, (0, 0, 255), 24)
                    cv2.circle(temp_img, (cell * router[0][1] + int(cell * 3), cell * router[0][0] + int(cell * 1)), 2,
                               (50, 255, 0), 24)
                else:
                    cv2.circle(temp_img, (cell * router[-1][1] + int(cell * 3), cell * router[-1][0] + int(cell * 1)),
                               2, (255, 0, 0), 24)
                    cv2.circle(temp_img, (cell * router[0][1] + int(cell * 3), cell * router[0][0] + int(cell * 1)), 2,
                               (255, 255, 255), 24)
                # 将图像显示在QT控件中
                qimg = QtGui.QImage(temp_img.data, temp_img.shape[1], temp_img.shape[0], temp_img.shape[1] * 3,
                                    QtGui.QImage.Format_BGR888)
                pixmap = QtGui.QPixmap.fromImage(qimg)
                self.camera_label.setPixmap(pixmap)
                time.sleep(0.2)  # 模拟运行延时
        self.lbl.setText('模拟运行完成，预计耗时' + str(int(time.time() - sim_timer)) + '秒')
        self.simbtn.setEnabled(True)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(sim.png);}")

    def Runcar(self):  # * 运行函数
        global result_final, boardmap, car_color_group, map_gui_image, True_Treas_Num, treasureinmap, treasureinmap_ori, Treas_distances
        self.bgmPlayer.play();
        calctimer = time.time()  # 计时器开始,播放音乐
        current_position = [18, 0]  # 当前位置存储
        Treasure_hit_route = [0, 0, 0]  # 宝藏点撞击方向/距挡板距离集合
        Treasure_if_hittable = 0  # 宝藏点是否可撞击
        final_point_route = 0  # 最终路程标记
        TreasureFinishList = [0] * 10  # 宝藏点完成情况记录
        TreasureRange = [[treasureinmap[0], 0]]  # 宝藏点撞击顺序集合,提前写入第一个宝藏点位置
        for i in range(len(treasureinmap_ori)):  # 遍历所有宝藏点寻找中心对称宝藏
            if treasureinmap_ori[i][1] == treasureinmap[0][1] and treasureinmap_ori[i][0] == treasureinmap[0][0]:
                TreasureRange[0][1] = i;
                break  # 找到第一个宝藏点的原始位置
        for i in range(len(treasureinmap)): boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 0  # 屏蔽所有宝藏点不能走
        camera = Camera(0)  # 提前打开相机
        camera.open()

        # TODO 运行8段路程，完成宝藏遍历
        for itel in range(len(result_final)):

            # TODO 更新宝藏撞击后当前位置
            if Treasure_if_hittable == 1:  # 如果上一个宝藏点可撞击,撞击后更新当前位置
                current_position[0] = TreasureRange[itel - 1][0][1];
                current_position[1] = TreasureRange[itel - 1][0][0]  # 撞击后更新当前位置

            # TODO 终点路径地图计算
            if True_Treas_Num == 3:  # * 如果已经遍历了3个宝藏点，准备离开迷宫
                print('进入终点路径计算');
                self.lbl.setText('运行中...前往终点');
                final_point_route = 1  # 标记为最终路程
            else:
                self.lbl.setText('运行中...前往第' + str(itel + 1) + '个宝藏点')

            # TODO 动态路径规划(ASTAR)
            boardmap[TreasureRange[itel][0][1]][TreasureRange[itel][0][0]] = 1  # 将目标宝藏点设置为可走
            boardmap[current_position[0]][current_position[1]] = 1  # 将当前位置设置为可走
            prj_map = Map(boardmap, current_position[0], current_position[1], TreasureRange[itel][0][1],
                          TreasureRange[itel][0][0])
            prj_map_result = astar(prj_map)  # A*路径规划
            prj_map_result.reverse()
            prj_map_result = np.asarray(prj_map_result)
            route_points = prj_map_result

            # TODO 计算本段路径拐点
            corners = [];
            print('本段路径总长度：', (len(route_points) - 1))
            for j in range(1, len(route_points) - 1, 1):
                dx = route_points[j + 1][0] - route_points[j - 1][0]
                dy = route_points[j + 1][1] - route_points[j - 1][1]
                delta = abs(dx * dx) + abs(dy * dy)
                if delta == 2: corners.append(route_points[j])  # else:continue
            corners.append(route_points[-1])  # 补上最终目标点

            # TODO 将路段数目发给下位机
            Full_route_numbers = len(corners) + 1 if Treasure_if_hittable == 1 else len(corners)  # 若本次需要撞击宝藏,路段数目+1
            RouteNumberCalc = 1 if Treasure_if_hittable == 1 else 0  # 路段数目计算存储
            if corners[0][0] == current_position[0] and corners[0][1] == current_position[
                1]: Full_route_numbers -= 1  # 如果首段路径为0(一般出现在不撞击宝藏时，车坐标正好处于路口情况下),不发送首段路径
            # * 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令[路段数目+1]
            serialapi.communicate(0xaa, 0xc1, eval(hex(Full_route_numbers)), 0x00, 0x00, 0x00, 0x00)
            # while True:
            #     if str(serialapi.recv)[0:14] == 'aa01c100000000': break;# 等待接收到回复信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.lbl.setText(
                '运行中...前往第' + str(itel + 1) + '个宝藏点,路段数目' + str(Full_route_numbers) + '已发送')
            self.sublbl.setText("前往宝藏点" + str(itel + 1) + "路段数目为" + str(Full_route_numbers))
            print("当前位置", str(current_position), "前往宝藏点", str(itel + 1),
                  "路段数目为" + str(Full_route_numbers))

            # TODO 将撞击指令单独发给下位机
            if Treasure_if_hittable == 1:  # 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令
                print("运动第1段路径开始啦")  # 发送控制指令
                serialapi.communicate(0xaa, 0xc2, 0x00, eval(hex(Treasure_hit_route[0])),
                                      eval(hex(Treasure_hit_route[1])), eval(hex(Treasure_hit_route[2] + 2)),
                                      eval(hex(0)))
                # while True:
                #     if str(serialapi.recv)[0:14] == 'aa01c200000000': break;# 等待接收到回复信号
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.lbl.setText('运行中...前往第' + str(itel + 1) + '个宝藏点前置上一宝藏撞击指令已发送')
                print("运动第1段路径前往", [TreasureRange[itel - 1][0][1], TreasureRange[itel - 1][0][0]],
                      "拐点,前进方向", Treasure_hit_route[0], "测距方向", Treasure_hit_route[1], "弧距0挡板距离",
                      Treasure_hit_route[2] + 2)

            # TODO 计算并发送每段路程控制指令并等待运行完成
            for index, corner in enumerate(corners):
                # * 计算拐点与当前位置的距离
                dx2 = corner[0] - current_position[0];
                dy2 = corner[1] - current_position[1]
                if (index + 1) < len(corners):  # 如果不是最后一个拐点
                    dx2_future = corners[index + 1][0] - corner[0];
                    dy2_future = corners[index + 1][1] - corner[1]
                    # * 计算转向弧距
                    TempArcMarker = 0  # * 临时记录弧线距离(默认为0)
                    if dy2_future > 0:
                        while True:  # ! 计算转向弧距
                            if not ((corner[1] - TempArcMarker) >= 0 and boardmap[corner[0]][
                                corner[1] - TempArcMarker] == 1): break
                            TempArcMarker += 1
                    elif dy2_future < 0:
                        while True:  # ! 计算转向弧距
                            if not ((TempArcMarker + corner[1]) < 19 and boardmap[corner[0]][
                                TempArcMarker + corner[1]] == 1): break
                            TempArcMarker += 1
                    elif dx2_future > 0:
                        while True:  # ! 计算转向弧距
                            if not ((corner[0] - TempArcMarker) >= 0 and boardmap[corner[0] - TempArcMarker][
                                corner[1]] == 1): break
                            TempArcMarker += 1
                    elif dx2_future < 0:
                        while True:  # ! 计算转向弧距
                            if not ((TempArcMarker + corner[0]) < 19 and boardmap[TempArcMarker + corner[0]][
                                corner[1]] == 1): break
                            TempArcMarker += 1
                else:  # 如果是最后一个拐点
                    dx2_future = -1919810;
                    dy2_future = -1919810;
                    TempArcMarker = 1

                # * 地图向右运行相关参数计算
                if dy2 > 0:  # 向右
                    Direction = 0;
                    SensDirection = 0 if index + 1 < len(corners) else 2;
                    single_route_steps = dy2
                    TempMarker = 0  # 临时记录挡板距离
                    if index + 1 == len(corners):
                        while True:  # ! 检测距挡板距离-最后路径段
                            if not ((corner[1] - TempMarker) >= 0 and boardmap[corner[0]][
                                corner[1] - TempMarker] == 1): break
                            TempMarker += 1
                    else:
                        while True:  # 检测距挡板距离-普通路径段
                            if not ((TempMarker + corner[1]) < 19 and boardmap[corner[0]][
                                TempMarker + corner[1]] == 1): break
                            TempMarker += 1
                # * 地图向左运行相关参数计算
                elif dy2 < 0:  # 向左
                    Direction = 2;
                    SensDirection = 2 if index + 1 < len(corners) else 0;
                    single_route_steps = abs(dy2)
                    TempMarker = 0  # 临时记录挡板距离
                    if index + 1 == len(corners):
                        while True:  # ! 检测距挡板距离-最后路径段
                            if not ((TempMarker + corner[1]) < 19 and boardmap[corner[0]][
                                TempMarker + corner[1]] == 1): break
                            TempMarker += 1
                    else:
                        while True:  # 检测距挡板距离
                            if not ((corner[1] - TempMarker) >= 0 and boardmap[corner[0]][
                                corner[1] - TempMarker] == 1): break
                            TempMarker += 1
                # * 地图向上运行相关参数计算
                elif dx2 < 0:  # 向上
                    Direction = 1;
                    SensDirection = 1 if index + 1 < len(corners) else 3;
                    single_route_steps = abs(dx2)
                    TempMarker = 0  # 临时记录挡板距离
                    if index + 1 == len(corners):
                        while True:  # ! 检测距挡板距离-最后路径段
                            if not ((TempMarker + corner[0]) < 19 and boardmap[TempMarker + corner[0]][
                                corner[1]] == 1): break
                            TempMarker += 1
                    else:
                        while True:  # 检测距挡板距离
                            if not ((corner[0] - TempMarker) >= 0 and boardmap[corner[0] - TempMarker][
                                corner[1]] == 1): break
                            TempMarker += 1
                # * 地图向下运行相关参数计算
                else:  # 向下
                    Direction = 3;
                    SensDirection = 3 if index + 1 < len(corners) else 1;
                    single_route_steps = dx2
                    TempMarker = 0  # 临时记录挡板距离
                    if index + 1 == len(corners):
                        while True:  # ! 检测距挡板距离-最后路径段
                            if not ((corner[0] - TempMarker) >= 0 and boardmap[corner[0] - TempMarker][
                                corner[1]] == 1): break
                            TempMarker += 1
                    else:
                        while True:  # 检测距挡板距离
                            if not ((TempMarker + corner[0]) < 19 and boardmap[TempMarker + corner[0]][
                                corner[1]] == 1): break
                            TempMarker += 1

                print("运动第" + str(RouteNumberCalc + 1) + "段路径开始啦")
                current_position = list(corner)  # 更新当前位置

                # TODO 接近宝藏时保持与宝藏距离防止撞到，并纠正当前坐标
                if index + 1 == len(corners):  # 最后一段路程
                    single_route_steps -= 2;
                    TempMarker -= 2  # 少走一步防止撞到宝藏,挡板距离也少2
                    Treasure_hit_route = [Direction, SensDirection, TempMarker - 1]  # 记录宝藏撞击路程的方向与距挡板距离[撞击需要]
                    if Direction == 0:
                        current_position[1] -= 2  # 修正向右当前坐标
                    elif Direction == 1:
                        current_position[0] += 2  # 修正向上当前坐标
                    elif Direction == 2:
                        current_position[1] += 2  # 修正向左当前坐标
                    elif Direction == 3:
                        current_position[0] -= 2  # 修正向下当前坐标
                if (itel + 1 == len(result_final) and index + 1 == len(corners)) or (
                        index + 1 == len(corners) and final_point_route == 1):  # 前往出口路程
                    single_route_steps = 4;
                    TempMarker = 5  # 出口路程固定为4步，冲出去
                if index + 1 == len(corners) and single_route_steps < 1:  # 每大段路程最后一段，若步数小于1，前进方向与上一段测距方向相反
                    Direction = 2 if last_SensDirection == 0 else (
                        3 if last_SensDirection == 1 else (0 if last_SensDirection == 2 else 1))
                else:
                    last_SensDirection = SensDirection  # 若不是每大段路程最后一段，记录本次路程的测距方向供之后使用

                # TODO 发送控制指令
                print("运动第" + str(RouteNumberCalc + 1) + "段路径前往" + str(corner) + "拐点,前进方向", Direction,
                      "测距方向", SensDirection, "弧距", TempArcMarker, "挡板距离", TempMarker - 1)
                if not (dx2 == 0 and dy2 == 0):  # 仅发送不为0的路径
                    serialapi.communicate(0xaa, 0xc2, eval(hex(RouteNumberCalc)), eval(hex(Direction)),
                                          eval(hex(SensDirection)), eval(hex(TempMarker - 1)),
                                          eval(hex(TempArcMarker - 1)))
                    # while True:
                    #     if str(serialapi.recv)[0:14] == 'aa01c2'+'{:02x}'.format(RouteNumberCalc)+'000000': break;# 等待接收到回复信号
                    serialapi.recv = str.encode('xxxxxxxxxxx');
                    RouteNumberCalc += 1  # 路径段数+1
                else:
                    print("运动第" + str(RouteNumberCalc + 1) + "段路径因距离为0跳过")
                self.lbl.setText('运行中...前往第' + str(itel + 1) + '个宝藏点,路段' + str(index + 1) + '已发送')

            # TODO 等待运行完成发送回复指令
            # while True:
            #     if str(serialapi.recv)[0:14] == 'aa210000000000': break;# 等待接收到回复识别请求信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            serialapi.communicate(0xaa, 0x01, 0x21, 0x00, 0x00, 0x00, 0x00)
            print("第" + str(itel + 1) + "个宝藏点已到达,当前位置", current_position)
            self.sublbl.setText("第" + str(itel + 1) + "个宝藏点已到达,当前位置" + str(current_position))
            Treasure_if_hittable = 0  # 重置宝藏撞击标志位

            # TODO 如果已经是最终路程，退出循环
            if final_point_route == 1: break

            # TODO 拍照识别
            if TreasureFinishList[TreasureRange[itel][1]] == 1:  # 预测过的宝藏不再识别
                self.sublbl.setText("调用预测数据，真宝藏...")
                print("调用预测数据，真宝藏...")
                # TODO 调用预测结果并判断是否发送撞击指令
                if car_color_group == "BLUE":
                    Treasure = "BlueTrue"
                    Treasure_if_hittable = 1  # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                    True_Treas_Num += 1  # 真宝藏数量+1
                elif car_color_group == "RED":
                    Treasure == "RedTrue"
                    Treasure_if_hittable = 1  # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                    True_Treas_Num += 1  # 真宝藏数量+1
            else:
                self.sublbl.setText("正在进行拍摄识别...")
                while 1:
                    ret, Treas_image = camera.read()  # 读取相机宝藏图像
                    if ret:
                        ret, Treas_image = camera.read()  # 读取相机宝藏图像
                        ret, Treas_image = camera.read()  # 读取相机宝藏图像
                        Treas_qimg = QtGui.QImage(Treas_image.data, Treas_image.shape[1], Treas_image.shape[0],
                                                  Treas_image.shape[1] * 3, QtGui.QImage.Format_BGR888)
                        Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
                        self.camera_label.setPixmap(Treas_pixmap);
                        break
                Treas_img_copy = Treas_image.copy()
                Treas_image = reshape_image_scan(Treas_image)[0]
                Treas_image, contours, yellow = Identify_cy.FindColorOne(Treas_img_copy, 1)  # 黄色
                Treas_image, contours, green = Identify_cy.FindColorOne(Treas_img_copy, 2)  # 绿色
                Treas_image, contours, blue = Identify_cy.FindColorOne(Treas_img_copy, 0)  # 蓝色
                Treas_image, contours, red = Identify_cy.FindRedOne(Treas_img_copy, contours)  # 红色
                # 蓝色1，黄色1，为蓝色真宝藏；红色1，绿色1，为红色真宝藏；蓝色1，绿色1，为蓝色假宝藏；红色1，黄色1，为红色假宝藏
                if blue == 1 and yellow == 1:
                    self.sublbl.setText('蓝色真宝藏');
                    print("蓝色真宝藏");
                    Treasure = "BlueTrue"
                elif red == 1 and green == 1:
                    self.sublbl.setText('红色真宝藏');
                    print("红色真宝藏");
                    Treasure = "RedTrue"
                elif blue == 1 and green == 1:
                    self.sublbl.setText('蓝色假宝藏');
                    print("蓝色假宝藏");
                    Treasure = "BlueFalse"
                elif red == 1 and yellow == 1:
                    self.sublbl.setText('红色假宝藏');
                    print("红色假宝藏");
                    Treasure = "RedFalse"
                else:
                    self.sublbl.setText('无宝藏');
                    print("无宝藏");
                    Treasure = "None"

                # TODO 根据识别结果判断是否发送撞击指令
                if car_color_group == "BLUE":
                    if Treasure == "BlueTrue":
                        Treasure_if_hittable = 1  # 宝藏点可撞击
                        self.sublbl.setText("可以撞击，已注入撞击指令")
                        print("可以撞击，已注入撞击指令")
                        True_Treas_Num += 1  # 真宝藏数量+1
                    else:
                        Treasure_if_hittable = 0  # 宝藏点不可撞击
                        print("不可以撞击，未注入撞击指令")
                elif car_color_group == "RED":
                    if Treasure == "RedTrue":
                        Treasure_if_hittable = 1  # 宝藏点可撞击
                        self.sublbl.setText("可以撞击，已注入撞击指令")
                        print("可以撞击，已注入撞击指令")
                        True_Treas_Num += 1  # 真宝藏数量+1
                    else:
                        Treasure_if_hittable = 0  # 宝藏点不可撞击
                        print("不可以撞击，未注入撞击指令")
            self.camera_label.setPixmap(map_gui_image)

            # TODO 根据识别结果动态规划下个目标点,存储宝藏点信息[0为未识别,1为预识别为真,-1为已撞击(假宝藏也算为已撞击/遍历)]
            TreasureFinishList[TreasureRange[itel][1]] = -1  # 标记已经到达的宝藏点为已撞击/遍历
            # ? 对称宝藏优化
            for i in range(len(treasureinmap_ori)):  # 遍历所有宝藏点寻找中心对称宝藏
                if TreasureFinishList[i] == 0:  # 仅更新未撞击/遍历的宝藏点,不覆盖先前推测结果
                    if treasureinmap_ori[i][1] == 18 - TreasureRange[itel][0][1] and treasureinmap_ori[i][0] == 18 - \
                            TreasureRange[itel][0][0]:
                        if Treasure_if_hittable == 1:
                            TreasureFinishList[i] = -1  # 如果宝藏点可撞击,标记已经到达的宝藏点中心对称点为对方真宝藏(已撞击/遍历)
                        else:  # 如果宝藏点不可撞击
                            if (car_color_group == "RED" and Treasure == "BlueTrue") or (
                                    car_color_group == "BLUE" and Treasure == "RedTrue"):  # 如果是对方真宝藏
                                TreasureFinishList[i] = 1  # 标记已经到达的宝藏点中心对称点为预识别己方真宝藏(待撞击)
                            if (Treasure == "RedFalse" or Treasure == "BlueFalse"):  # 如果是假宝藏
                                TreasureFinishList[i] = -1  # 标记已经到达的宝藏点中心对称点为假宝藏(已撞击/遍历)
            # ? 同象限宝藏优化
            for i in range(len(treasureinmap_ori)):  # 遍历所有宝藏点寻找同象限宝藏
                if TreasureFinishList[i] == 0:  # 仅更新未撞击/遍历的宝藏点,不覆盖先前推测结果
                    if ((treasureinmap_ori[i][0] - 9) / abs(treasureinmap_ori[i][0] - 9) == (
                            TreasureRange[itel][0][0] - 9) / abs(TreasureRange[itel][0][0] - 9)) and (
                            (treasureinmap_ori[i][1] - 9) / abs(treasureinmap_ori[i][1] - 9) == (
                            TreasureRange[itel][0][1] - 9) / abs(TreasureRange[itel][0][1] - 9)):
                        if (car_color_group == "RED" and (Treasure == "RedTrue" or Treasure == "RedFalse")) or (
                                car_color_group == "BLUE" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")):
                            TreasureFinishList[i] = -1  # 标记同象限不同色宝藏点为假宝藏(已撞击/遍历)
                        elif (car_color_group == "RED" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")) or (
                                car_color_group == "BLUE" and (Treasure == "RedTrue" or Treasure == "RedFalse")):
                            for j in range(len(treasureinmap_ori)):  # 遍历所有宝藏点寻找同象限宝藏的对称点
                                if 18 - treasureinmap_ori[i][0] == treasureinmap_ori[j][0] and 18 - \
                                        treasureinmap_ori[i][1] == treasureinmap_ori[j][1]:
                                    TreasureFinishList[j] = -1  # 若本次为对方颜色宝藏，标记同象限不同色宝藏点的对称点为假宝藏(已撞击/遍历)
            # ? 动态计算下一目的地
            Total_Visited_Treasure = 0  # 已经遍历的宝藏点数量
            for i in range(len(TreasureFinishList)):  # 检查是否所有点都去过
                if TreasureFinishList[i] == -1: Total_Visited_Treasure += 1
            if True_Treas_Num == 3 or itel + 2 == len(result_final) or Total_Visited_Treasure == 8:
                TreasureRange.append([[18, 0], 10]);print("经计算，下个目标点是终点")  # 若已经完成三个寻宝或8次运动,将终点存入队列
            else:
                Temp_Next_Treasure = [1000, 1000]  # 临时变量,存储下一个宝藏点
                for i in range(len(treasureinmap_ori)):  # 遍历所有宝藏点寻找下一运动点
                    if TreasureFinishList[i] != -1:  # 找出未遍历的点
                        if (Treas_distances[TreasureRange[itel][1]][i] < Temp_Next_Treasure[0]):  # 找出距离最近的点
                            Temp_Next_Treasure[0] = Treas_distances[TreasureRange[itel][1]][i];
                            Temp_Next_Treasure[1] = i  # 存储距离最近的点
                if Temp_Next_Treasure != [1000, 1000]: TreasureRange.append(
                    [treasureinmap_ori[Temp_Next_Treasure[1]], Temp_Next_Treasure[1]])  # 下一个宝藏点存入队列
                print("经计算，下个目标宝藏点是：", [TreasureRange[itel + 1][0][1], TreasureRange[itel + 1][0][0]])

            for i in range(len(treasureinmap)): boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 0  # 屏蔽所有宝藏点不能走

            time.sleep(3)

        # TODO 到达终点
        self.sublbl.setText('Misson Complete');
        self.lbl.setText('完成运行,耗时' + str(int(time.time() - calctimer)) + '秒')
        camera.release()  # 释放相机资源
        pixmap = QtGui.QPixmap('layla.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)
        self.bgmPlayer.stop()  # 停止播放音乐


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())