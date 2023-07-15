#-*-coding:utf-8-*-
# Function: Main program
#? 主程序
#TODO Version 0.1.20230714
#! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py
#* Thread 利用情况：Thread-0 UART通信
#* Thread 利用情况：Thread-1 路径规划线程
#* Thread 利用情况：Thread-2 地图扫描线程
import sys,cv2,time,threading # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton # 导入PyQt5模块
from PyQt5 import QtGui,QtCore # 导入PyQt5GUI模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours # 导入地图寻路部分
from MapScan import  reshape_image_scan, detect, find,affine_transformation # 导入地图扫描部分
from Astar import Map,MapNode,astar # 导入A*算法部分
import numpy as np # 导入Numpy模块
import serialapi # 导入串口模块

CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 720 # 设定的相机取样像素参数
result_final = [] # 寻路结果存储

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

class Example(QWidget): #TODO 主窗口类


    def __init__(self): #* 初始化函数
        super().__init__()

        self.initUI()


    def initUI(self): #* 界面初始化函数

        self.lbl = QLabel('系统初始化进行中', self)
        self.lbl.move(0, 30)
        self.lbl.setFixedSize(600, 30)
        self.lbl.setAlignment(QtCore.Qt.AlignCenter) # 修改字体居中
        self.lbl.setFont(QtGui.QFont("Arial", 16)) # 修改字体大小为16px

        # 启动按钮
        self.stbtn = QPushButton('拍摄藏宝图', self)
        self.stbtn.clicked.connect(self.startup_thread) # 修改按钮行为为启动特定函数“Startup”
        self.stbtn.move(40, 80)
        self.stbtn.setFixedSize(160, 50)
        # 运行按钮
        self.runbtn = QPushButton('启动AutoCar运行', self)
        self.runbtn.clicked.connect(self.runcar_thread) # 修改按钮行为为启动小车运行函数
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
            serialapi.communicate(0xaa,0xa1,0x00,0x00,0x00,0x00,0x00) # 发送启动信号
            while True:
                if serialapi.recv == 'aa 01 a1 00 00 00 00': break;# 等待接收到启动信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.lbl.setText('串口通信通路正常,系统初始化完成')


    def startup_thread(self): #* 启动路径规划线程
        t = threading.Thread(target=self.Startup)
        t.start()


    def runcar_thread(self): #* 启动运行线程
        t = threading.Thread(target=self.Runcar)
        t.start()


    def Startup(self): #* 启动函数
        global result_final
        camera = Camera(1) # 打开相机
        camera.open()
        # 等待调整相机并拍照
        last_image = None
        start_time = time.time()
        while time.time() - start_time < 8:
            ret, image = camera.read() # 读取相机图像
            # 将相机图像显示在控件中
            qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0],image.shape[1]*3, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.lbl.setText('调节时间剩余'+str(int(8-time.time() + start_time))+"秒，请注意")
            last_image = image

        #? 照片扫描加纠偏部分开始
        self.lbl.setText('照片扫描纠偏中...')
        image, new_width, new_height = reshape_image_scan(last_image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image, contours, hierachy = detect(image)
        rec ,img= find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0],img.shape[1]*3, QtGui.QImage.Format_BGR888)
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
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 地图扫描输出数组部分结束

        self.lbl.setText('路径规划进行中...')
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
        # print("怎么移动：", result_final)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        # 界面更新
        self.stbtn.setEnabled(False)
        self.runbtn.setEnabled(True)
        self.lbl.setText('路径规划完成,请按下Enter运行...')


    def Runcar(self): #* 运行函数
        global result_final
        current_position = [18,-1] # 当前位置存储

        #TODO 运行8段路程，完成宝藏遍历
        for itel in range(len(result_final)):
            self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点')

            #TODO 计算拐点
            route_points = result_final[itel]
            corners = []
            print(len(route_points) - 1)
            for j in range(1,len(route_points) - 1,1):
                dx = route_points[j+1][0] - route_points[j-1][0]
                dy = route_points[j+1][1] - route_points[j-1][1]
                delta = abs(dx * dx) + abs(dy * dy)
                if delta == 2:corners.append(route_points[j])
                else:continue
            corners.append(route_points[-1]) # 补上最终目标点

            #TODO 将路段数目发给下位机
            # serialapi.communicate(0xaa,0xc1,hex(len(corners)),0x00,0x00,0x00,0x00)
            # while True:
            #     if serialapi.recv == 'aa 01 c1 00 00 00 00': break;# 等待接收到回复信号
            # serialapi.recv = str.encode('xxxxxxxxxxx')
            # self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点,路段数目'+str(len(corners))+'已发送') 
            print("前往宝藏点",str(itel+1),"路段数目为"+str(len(corners)))

            #TODO 计算并发送每段路程控制指令并等待运行完成
            for index,corner in enumerate(corners):
                # 计算拐点与当前位置的距离
                dx2 = corner[0] - current_position[0];dy2 = corner[1] - current_position[1]
                if dy2 >0: # 向右
                    Direction = 'D';single_route_steps = dy2
                elif dy2 <0: # 向左
                    Direction = 'A';single_route_steps = abs(dy2)
                elif dx2 <0: # 向上
                    Direction = 'W';single_route_steps = abs(dx2)
                elif dx2 >0: # 向下
                    Direction = 'S';single_route_steps = dx2
                print("第"+str(index+1)+"段路径前往"+str(corner)+"拐点,(测距/前进)方向",Direction,"路径长度",single_route_steps)
                current_position = corner # 更新当前位置
                # 发送控制指令
                # serialapi.communicate(0xaa,0xc2,hex(index+1),hex(Direction),hex(Direction),0x00,hex(single_route_steps))
                # while True:
                #     if serialapi.recv == 'aa 01 c2 '+str(hex(Direction))+' 00 00 00': break;# 等待接收到回复信号
                # serialapi.recv = str.encode('xxxxxxxxxxx')
                # self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点,路段数目'+str(len(corners))+'已发送') 
            print("第"+str(itel+1)+"个宝藏点已到达,当前位置",current_position)
            time.sleep(1)

            # TODO 拍照识别
            # 等待填写

            # TODO 发送撞击指令与否
            # 等待填写


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())