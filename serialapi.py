#-*-coding:utf-8-*-
# Function: Serial API
#? 串口通信API
#TODO Version 0.1.20230715
#! 依赖项目：pyserial
#! 被引用：main.py
import serial,time
import serial.tools.list_ports
If_Serial_Open = False


SerialPortList = list(serial.tools.list_ports.comports())
if len(SerialPortList) <= 0:
    print ("The Serial port can't find!")
else:
    serialName =list(SerialPortList[0])[0]
    ser = serial.Serial(serialName, 115200, timeout=None) # 打开串口，已重定向
    # 校验位设置
    ser.parity = serial.PARITY_NONE
    # 停止位设置
    ser.stopbits = serial.STOPBITS_ONE
    # 数据位设置
    ser.bytesize = serial.EIGHTBITS
    print ("Now Port Using >",ser.name)
    If_Serial_Open = ser.name
    ser.eol=b'\r\n'
recv = str.encode('xxxxxxxxxxx')       # ? UART回传数据全局存储

'''  下位机数据传输封装  '''
def crc16(buf): #* CRC16校验
    crc = 0xffff  # 初始化,这里的CRC16是小端模式,高位在后,低位在前,所以初始值为0xffff
    for b in buf:  # 逐位移出数据
        crc ^= b  # 与CRC寄存器进行异或运算
        for i in range(8):  # 循环计算每一位
            if crc & 1:  # 判断右移出的是不是1，也就是最低位是不是1
                crc = (crc >> 1) ^ 0xa001  # 将CRC寄存器右移一位，如果右移出的是1，那么就与多项式A001进行异或运算，否则直接右移
            else:  # 如果最低位不是1，那么就直接右移
                crc >>= 1  # 右移一位
    return crc # 返回最终计算出来的CRC值,假如buf中的数据为0x00 0x07 0x01 0x00 0x01 0x01 0x00 0x00，那么返回的CRC值为0x01 0x01。
    # 今天晚上吃什么
def communicate(map,length,step_final,hitdirection,hitdistance):
    # 通信协议
    # 起始字节：一个字节，固定为0xaa，用于标记数据帧的开始。
    # 数据长度：一个字节，表示数据内容的长度。
    # 指令码：一个字节，表示数据帧的指令码，用于标识数据帧的类型。
    # 数据内容：长度为数据长度字节所表示的长度，包含实际的数据内容。
    # CRC校验码：两个字节，表示数据内容的CRC校验码，用于检测数据传输过程中是否出现错误。
    # 结束字节：一个字节，固定为0xff，用于标记数据帧的结束。
    # 以16进制形式发送上面的数据
    # 创建一个数组buf，用于存放数据内容
    buf = []
    # 创建一个data数组，用于存放数据内容
    data = []

    
    ''' 数据编辑'''
    #? 起始字节部分
    data.append(0xaa)  # 起始字节存入数组data
    #? 数据长度/指令码/数据内容部分
    #! map=1表示发送路径信息
    if (map == 1):
        #? 数据长度
        data.append(length)

        #? 指令码 hitdirection表示发送摄像头方向0x00到0x03分别对应0x05到0x08
        if (hitdirection == 0x00):
            data.append(0x05)  
        elif (hitdirection == 0x01):
            data.append(0x06)
        elif (hitdirection == 0x02):
            data.append(0x07)
        elif (hitdirection == 0x03):
            data.append(0x08)

        #? 数据内容部分，发送路径信息:
        for i in range(0,length):  
            data.append(step_final[0][i])
            buf.append(step_final[0][i])  # 数据内容存入数组buf
    
    #! map=0表示发送撞击信息
    else: 
        #? 数据长度
        data.append(0x02)

        #? 指令码 0x02表示这是撞击信息:
        data.append(0x02)

        #? 数据内容部分，发送撞击方向:
        data.append(hitdirection)
        buf.append(hitdirection)  # 数据内容存入数组buf

        data.append(hitdistance)
        buf.append(hitdistance)  # 数据内容存入数组buf
    #? CRC校验部分
    # crc校验码
    # crc校验码的计算方法是把前面的数据内容（包括起始字节、数据长度字节、指令码字节和CRC校验码字节）的每一个字节相加，然后取反加1，最后取低两个字节。
    # 例如，发送数据为0x00 0x07 0x01 0x00 0x01 0x01 0x00 0x00，CRC校验码为0x01 0x01。计算过程如下：0x00 + 0x07 + 0x01 + 0x00 + 0x01 + 0x01 + 0x00 + 0x00 = 0x0a
    # 0x0a取反加1 = 0xf6 + 0x01 = 0xf7 ; 0xf7取低两个字节 = 0x01 0x01；最后发送crc校验码
    crc = crc16(buf)  # 计算crc校验码
    
    crc_low = crc & 0xff  # 取crc的低8位
    data.append(crc_low)  

    crc_high = (crc & 0xff00) >> 8  # 取crc的高8位
    data.append(crc_high)

    #? 结束字节部分
    data.append(0x55)  # 结束字节

    ''' 数据编辑完成，开始发送 '''
    print("本次发送内容为:", data)
    # 发送数据
    ser.write(data)  # 发送数据

''' Thread-0 UART扫描读取封装 '''
def uartRx(): # UART扫描读取封装
    
    global recv, ser
    
    recv = str.encode('xxxxxxxxxxx')
    
    print('串口就绪，开始接收')
    while True:

        # 获得接收缓冲区字符
        count = 0

        if count == 0:

            # 读取内容并回显
            recv = ser.read(4)
            recv = recv.hex()
            print(recv)
            ser.flushInput()  # 清空接收缓冲区
