import sensor, image, time, lcd, math, struct
from fpioa_manager import fm
from machine import UART
from collections import OrderedDict

# ================= 📝 参数填空区 =================
# 1. 这里填你打印的二维码黑边实际长度 (单位: mm)
TAG_SIZE = 80    
# 2. 焦距常数 (默认280，如果距离不准可微调)
F_CONST  = 280   
# ===============================================

# --- 数据打包类 (无需改动) ---
class Uart_SendPack():
    def __init__(self, packmsg, dataformat):
        self.msg = packmsg
        self.sendformat = dataformat
    def calculate_BCC(self, datalist, datalen):
        ref = 0
        for i in range(datalen):
            ref = (ref ^ datalist[i])
        return ref & 0xff
    def pack_BCC_Value(self):
        tmp_list = list(self.msg.values())
        tmp_packed = struct.pack(self.sendformat, *tmp_list)
        return self.calculate_BCC(tmp_packed, len(tmp_packed)-2)
    def get_Pack_List(self):
        tmplist = list(self.msg.values())
        return struct.pack(self.sendformat, *tmplist)

# 定义发送数据包: Head, W, H, X, Y, Z, Bcc, End
send_pack1_msg = OrderedDict([('Head',0xCC),('Cam_W',320),('Cam_H',240),
                              ('x',0),('y',0),('z',0),('Bcc',0),('End',0xDD)])
send_pack1 = Uart_SendPack(send_pack1_msg, "<B5H2B")

def send_data(x, y, z):
    # 发送给 STM32: x=偏移量, y=ID, z=距离
    send_pack1.msg['x'] = int(x)
    send_pack1.msg['y'] = int(y)
    send_pack1.msg['z'] = int(z)
    send_pack1.msg['Bcc'] = send_pack1.pack_BCC_Value()
    return send_pack1.get_Pack_List()

# --- 初始化 ---
lcd.init()
sensor.reset()
sensor.reset(freq=24000000, dual_buff=1)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.set_auto_gain(True)      # Tag识别必须开增益
sensor.set_auto_whitebal(True)  # Tag识别必须开白平衡
sensor.skip_frames(time=1000)
clock = time.clock()

# 串口初始化
fm.register(0, fm.fpioa.UART1_RX)
fm.register(1, fm.fpioa.UART1_TX)
uart1 = UART(UART.UART1, 115200)

# 巡线设置
color_thresholds = [(0, 15, -128, 127, -128, 127)] # 黑色
line_roi = (40, 200, 240, 40) # 只看脚下

while True:
    clock.tick()
    try:
        img = sensor.snapshot()
        
        # === 1. 找 Tag (高优先级) ===
        tags = img.find_apriltags(families=image.TAG36H11)
        tag_found = False
        
        if tags:
            tag_found = True
            for tag in tags:
                img.draw_rectangle(tag.rect(), color=(0, 255, 0), thickness=3)
                
                # 计算距离 (cm)
                pixel_width = tag.rect()[2]
                distance_cm = (TAG_SIZE * F_CONST) / pixel_width / 10
                # 计算偏移 (画面中心160)
                x_offset = tag.cx() - 160
                
                # 发送: x=偏移(+160转正数), y=TagID, z=距离
                uart1.write(send_data(x_offset + 160, tag.id(), distance_cm))
                img.draw_string(0, 0, "ID:%d Dist:%.1f" % (tag.id(), distance_cm), scale=2, color=(0,255,0))
                break 

        # === 2. 没 Tag 就巡线 (低优先级) ===
        if not tag_found:
            blobs = img.find_blobs(color_thresholds, roi=line_roi, pixels_threshold=100, merge=True)
            if blobs:
                max_blob = max(blobs, key=lambda b: b.pixels())
                img.draw_rectangle(max_blob.rect(), color=(255, 0, 0))
                img.draw_cross(max_blob.cx(), max_blob.cy(), color=(255, 0, 0))
                
                # 发送: x=CX, y=CY(>200), z=面积
                # STM32 靠 y > 100 来判断这是巡线数据
                uart1.write(send_data(max_blob.cx(), max_blob.cy(), max_blob.pixels()))
            
        lcd.display(img)
    except Exception as e:
        pass