import serial
import serial.tools.list_ports
import time
import threading
from enum import Enum
import os
import datetime
class updateState(Enum):
    start = 1
    writeT = 2
    writeEnter = 3
    writeUartUpgradeapp = 4
    uploadingFile = 5

class Uploader:
    def __init__(self):
        self.state = updateState.start
        # self.buildfileName = "./build/fmt_chuanyun_f2s.bin"
        self.fileName = os.path.dirname(os.path.abspath(__file__))+"/package_bin/fmt_chuangyun_f2s_pkg.bin"
        # self.fileName = "/home/robot/workspace/feizou/code/fmt-gitee/FMT-Firmware/target/coolfly/chuanyun_f2s/package_bin/app-SKY-ZZ-2023-02-10.bin"
        
    def openSerial(self):
        self.ser = serial.Serial("/dev/ttyUSB0", 460800)    # 打开COM17，将波特率配置为115200，其余参数使用默认值
        if self.ser.isOpen():                        # 判断串口是否成功打开
            print("打开串口成功。")
            print(self.ser.name)    # 输出串口号
        else:
            print("打开串口失败。")
    def writeFileData(self):
        print(">>>>>>>>>>>>>>>>>>>write file\n")
        file = open(self.fileName,mode='rb+')
        data = file.read()
        self.ser.write(data)
    def startUploadFile(self):
        t = threading.Thread(target=self.writeFileData,args=())
        t.start()
    def reboot(self):
        self.ser.write(b"reboot\r\n")
    def update(self):
        # time.gmtime(os.path.getmtime(self.buildfileName))
        
        updating = True;
        while updating:
            com_input = self.ser.readline()
            strData = com_input.decode()
            print(strData)
            if self.state == updateState.start:
                self.ser.write(b't')
                if strData.find("Bootloader") >= 0:
                    self.state = updateState.writeT
            if self.state == updateState.writeT:
                
                if strData.find("upgrade start") >= 0:
                    # print(">>>>>>>>>>>>>>>>>>>writeEnter")
                    self.ser.write(b'\r\n')
                    self.state = updateState.writeEnter
            if self.state == updateState.writeEnter:
                if strData.find("uart_upgradeapp") >= 0:
                    # print(">>>>>>>>>>>>>>>>>>>write uart_upgradeapp\n")
                    self.ser.write(b"uart_upgradeapp\r\n")
                    self.state = updateState.writeUartUpgradeapp
            if self.state == updateState.writeUartUpgradeapp:
                if strData.find("interrupt") >= 0:
                    self.startUploadFile()
                    self.state = updateState.uploadingFile
            # if self.state == updateState.uploadingFile:
            #     if strData.find("cpu2 start") >= 0:
            #         break
uploader = Uploader();
uploader.openSerial();
uploader.reboot();
print("按复位键")
# uploader.startUploadFile()
uploader.update();