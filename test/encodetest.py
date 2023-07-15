'''  下位机数据传输封装  '''
def communicate(RB0,RB1,RB2,RB3,RB4,RB5,RB6): # 下位机数据传输函数
    global ser
    RB7 = 0xff - (sum([RB0,RB1,RB2,RB3,RB4,RB5,RB6]) & 0xff)
    hexcomm = '{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}'.format(RB0, RB1, RB2, RB3, RB4, RB5, RB6, RB7)
    byte_text = bytes.fromhex(hexcomm)
    print(byte_text)

communicate(0x4a,0x17,0x00,0x00,0x00,0x00,0x00)