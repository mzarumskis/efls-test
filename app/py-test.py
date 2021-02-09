'''
Created on Oct 14, 2020

@author: mzaru
'''
import serial
from serial.tools.list_ports import comports
import platform
import time
import sys
import binascii
import crc8
import threading
import os
from queue import Queue 
from pickle import TRUE
import PySimpleGUI as sg
from matplotlib._layoutbox import align

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

import matplotlib

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import datetime as dt

#from Tkinter import *

version = "V-0.0.1"

PERIODIC_DATA_RESPONCE = 1

FOTA_BLOCK_SZIE_IN_BYTES = 64

DELAY_AFTER_RECEIVE = 0.05
LOOP_IN_MS = 10
NO_RECEIVE_TIMOUT_IN_S = 2 * LOOP_IN_MS

isSwitchedToFota = 0
noPacketTimeout = 0
frameErrorCounter = 0 
RS485_BOUD_RATE_BOOT_ONLY = 230400
RS485_BOUD_RATE_APP = 19200

#Command definitions

WRITE_FOTA_RECORD = 4
WRITE_FOTA_HEADER = 11

def open_serial(port, boudR):
    ser = serial.Serial(port, boudR, timeout=0.1)
    return ser

def AddCrc(data):
  crc = 0x0000
  for n in data[2:]:  
      crc = crc ^ (n << 8) 
      for bitnumber in range(0,8):
        if crc & 0x8000 : 
            crc =  crc  ^  (0x1070 << 3)
        crc = ( crc << 1 )
  crc = crc >> 8
  return crc & 0xFF

def frame_BOOT_Commanded():
    global serTerminal
    buff = bytearray(b'\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E')
   
    print("Data->TX {0}".format(buff.hex().upper()))
    serTerminal.write(buff) 
     
    
# switch to SLAVE mode
def frame_02():
    buff = bytearray(b'\x31\xFE')
    buff.append(2)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    serTerminal.write(buff)    

# get Build    
def frame_09():
    buff = bytearray(b'\x31\xFE')
    buff.append(9)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    serTerminal.write(buff)    

def frame_04_writeRecordToFlash(recordIdx,size ,data):
    buff = bytearray(b'\x31\xFE')
    buff.append(WRITE_FOTA_RECORD)
    buff.append(size+3)
    buff.append(recordIdx & 0xFF)
    buff.append((recordIdx >> 8) & 0xFF)
    buff.append(size)
    
    for b in data:
        buff.append(b)
        
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    serTerminal.write(buff)    

def frame_11_writeFotaHeader(version):
    buff = bytearray(b'\x31\xFE')
    buff.append(WRITE_FOTA_HEADER)
    buff.append(3)
    buff.append(version[0])
    buff.append(version[1])
    buff.append(version[2])
              
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    serTerminal.write(buff)    


def getDataFromSerial():
    data = serialQ.get()
    if data:
        #time.sleep(1/LOOP_IN_MS) #delay 50mS 
        return data
    
    return ""   

def getDataFromSerialWithTimeout(timeOut):
    data = serialQ.get(timeout = timeOut)
    if data:
        #time.sleep(1/LOOP_IN_MS) #delay 50mS 
        return data
    
    return ""  


def getAnyFromRS485():
    if serialQ.get():
        return     
    time.sleep(1/LOOP_IN_MS) #delay 50mS
     
def readSerial(ser, serialQ):
    global stop_threads
    
    while (ser.isOpen()):
        rawdata = ser.read(256)
        
        if rawdata:
            serialQ.put(rawdata)
           
        if stop_threads:
            break    
        time.sleep(1/LOOP_IN_MS) #delay 50mS

def isPackedValid(data):
    
    if len(data) < 5:
        return False
    if len(data) < data[3]+5:
        return False
    if len(data) < data[3]+3:
        return False
    if data[0] != 0x31:
        return False 
    
    return packetCrcCheck(data)

def isBuildAckFrame(data):
    if data[2] == 10:
        return True
    return False
def isSwticToSlavedAckFrame(data):
    if data[2] == 3 and data[4] == 2 :
        return True
    return False

def isWriteRecorddAckFrame(data):
    if data[2] == 5 and data[4] == 4 :
        return True
    return False
      
def packetCrcCheck(data):
    crc = 0x0000
    for n in data[2:data[3]+4]:  
      crc = crc ^ (n << 8) 
      for bitnumber in range(0,8):
        if crc & 0x8000 : 
            crc =  crc  ^  (0x1070 << 3)
        crc = ( crc << 1 )
    crc = crc >> 8
    if crc & 0xFF == data[data[3]+4]:
        return True
    else:
        return False

def switchToFwUpdate():
    dummy =""
    while True:
        frame_BOOT_Commanded()
        try:
            data = getDataFromSerialWithTimeout(0.1)
            print("data")
            if isPackedValid(data):
               return True 
        except :
            dummy="e"
            print("Timeout")
   
   # data = getDataFromSerial()

def getBuild():
    global fwBuild
    tryCount = 3
    while(tryCount):
        getAnyFromRS485()
        time.sleep(DELAY_AFTER_RECEIVE) #delay 
        frame_09()
        data = getDataFromSerial()
        print("Data->RX {0}".format(data.hex().upper()))
      
      
        tryCount = tryCount - 1
        if isPackedValid(data):
             if isBuildAckFrame(data):
                 print("Build {0}".format(data[4:data[3]+3]))
                 fwBuild = "{0}".format(data[4:data[3]+3])
                 break
        else:
            print("Packet ERROR")
            

def switchToSlaveMode():
        
    tryCount = 5
    getAnyFromRS485()
    while(tryCount):
        
        time.sleep(DELAY_AFTER_RECEIVE) #delay 
        frame_02()
        try:
            data = getDataFromSerialWithTimeout(0.5)
            print("Data->RX {0}".format(data.hex().upper()))
            tryCount = tryCount - 1
            if isPackedValid(data):
                 if isSwticToSlavedAckFrame(data):
                     print("Slave Mode Entered")
                     break
            else:
                print("Packet ERROR")
        except :
  
           print("Timeout")    
            
def waitWriteRecordAck(idx):
    try:
        data = getDataFromSerialWithTimeout(0.5)
        print("Data->RX {0}".format(data.hex().upper()))
        if isPackedValid(data):
           if isWriteRecorddAckFrame(data):
               idxAck = (data[6]<<8) + data[5]
               if idxAck == idx:
                   return True
        return False
    except :
        print("No Data Timout")  
        return False     
def waitAckPacket():
     try:
        data = getDataFromSerialWithTimeout(0.5)
        print("Data->RX {0}".format(data.hex().upper()))
      
    
        return True
     except :
        print("No Data Timout")  
        return False     
    
    
    
def getBinFile(dir_path):
  
    return os.path.basename(dir_path)

def getVersionFromFileName(fileName):
    if fileName != "":
        print("File found: {0}".format(fileName))
        name = fileName.split("_")
        version = name[1].split(".")
        data = bytes.fromhex(version[0]+ version[1]+version[2])
        return data
        
    return ""
        
        
def fwUploadTask(updateFotaHeaderOnly, filePath, win):
    global progress
    print("Opne file")
    fileName = getBinFile(filePath)
    if filePath != "":
        
        if not updateFotaHeaderOnly:
            
                file = open(filePath, "rb")
                recordIdx =0
                fileSize = len(file.read())
                print("File found: {0} szie {1}".format(fileName, fileSize))
                file.seek(0)      
                while True:
                    data = file.read(FOTA_BLOCK_SZIE_IN_BYTES)
                    if data:
                        writeOk = False
                        
                        while not writeOk:
                                
                            frame_04_writeRecordToFlash(recordIdx, len(data), data)
                            if waitWriteRecordAck(recordIdx):
                               writeOk = True
                            else:
                                print("Write Error {0}".format(recordIdx)) 
                            
                        recordIdx = recordIdx +1
                        bytesSent = (recordIdx * FOTA_BLOCK_SZIE_IN_BYTES)
                        progress = round((bytesSent/fileSize) * 100)
                        print("Sent {0}%".format(progress))
                        #win.FindElement('fwUpdateProgress').UpdateBar(progress, 100)
                                     
                    else:
                        version = getVersionFromFileName(fileName)    
                        frame_11_writeFotaHeader(version)   
                        waitAckPacket()
                        break
        else:
            version = getVersionFromFileName(fileName)    
            frame_11_writeFotaHeader(version)   

def getDataFromSensor(win):
    dummy =""
   
    try:
        data = getDataFromSerialWithTimeout(0.2)
        
        if isPackedValid(data):
           #print("Data->RX {0}".format(data.hex().upper())) 
           if data [2] == PERIODIC_DATA_RESPONCE:
               print("Temp: {0}; TempStatus: {1}; Photo: {2}; CAP1: {3}pF; CAP2: {4}pF; Level1: {5}; Level2: {6}".format(data[4], data[5], data[6], data[7]<<8 |data[8], data[9]<<8|data[10], data[13]<<8|data[14], data[15]<<8|data[16]))
               dataMap["Cap1"] = data[7] << 8 | data[8]
               dataMap["Cap2"] = data[9]<<8 | data[10]
               dataMap["level1"] = data[13]<<8 | data[14]
               dataMap["level2"] = data[15]<<8 | data[16]
               dataMap["Temp"] = data[4]
               dataMap["Photo"] = data[6]
               dataMap["TempError"] = data[5]
                 
               
        return True 
    except :
        dummy="e" 
 
        
        return False    
    
    


def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return figure_canvas_agg                        

def windows_ini():
    matplotlib.use("TkAgg")
    frame2_layout = [
        [sg.Text("                                                          ")],
        [sg.Text("Choose a file: "), sg.FileBrowse("File", key="-FILE-")],
        [sg.Button("UPDATE")],
        [sg.ProgressBar(orientation="horizontal",max_value=10,size=(10, 20), key = "fwUpdateProgress"), sg.Text("0%  ", key = "progress")],
        ]
    information_column = [
                            [sg.Checkbox("Boot Only", key = "boot-checkbox"), sg.Combo("",size =(10,1), key = "port-list")],
                            [sg.Button("CONNECT")],
                            [sg.Text("Fw Version:"), sg.Text("____________", key="-version-") ],
                            [sg.Text("Build:"), sg.Text("______________________________", key = "-build-")],
                            [sg.HSeparator(color = "White"),],
                            [sg.Text("                                                          ")],
                            [sg.Text("Temp:"),sg.Text("####", key="-temp-") ],
                            [sg.Text("Status"),sg.Text("####", size = (6,1),key="-tempStatus-") ],
                            [sg.Text("")],
                            
                            [sg.Text("CAP1:"),sg.Text("####", key="-cap1-") ,sg.Text("pF")],
                            [sg.Text("CAP2:"),sg.Text("####", key="-cap2-") ,sg.Text("pF")],
                            [sg.Text("")],
                             [sg.Text("Photo:"),sg.Text("ON", size = (6,1), key="-photo-") ],
                           
                            [sg.Text("")],
                            [sg.Frame(layout = frame2_layout,  title="FW")],
                         ]
   
    
    information_column2_1 = [
                            [sg.ProgressBar(orientation="vertical",max_value=10,size=(20, 20)), sg.Canvas(key="-CANVAS-")],
                           
                            [sg.ProgressBar(orientation="vertical",max_value=100,size=(20, 20)), sg.Canvas(key="-CANVAS2-")],
                           
                           
                         ]
    
 
    
    layout = [ 
                [sg.Frame(layout = information_column, title="frame1",  vertical_alignment = "top", key= "frame1"),sg.Column(information_column2_1)] ,
                
               
               
             ]

    #x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
    #y= np.array ([0,10,20,30,40,50,60,70,80,90,100])
    print("Starting")
    
     # Create the window
    window = sg.Window("EFLS-D2 Test " + version, layout, size=(1360,768),finalize=True,
    element_justification="left",
    font="Helvetica 12",)
    return window
    
def plot_ini(window):
    global fig
    global subPlot
    fig = Figure(figsize=(4.9,2.9))
    subPlot = fig.add_subplot(111)
    subPlot.axis([0, 10, 0, 100])
    #a.scatter(100,10,color='red')
    #a.plot(p, range(2 +max(x)),color='blue')
    #a.invert_yaxis()

    subPlot.set_title ("CAP1", fontsize=16)
    subPlot.set_ylabel("Y", fontsize=14)
    subPlot.set_xlabel("t", fontsize=14)
    br = draw_figure(window["-CANVAS-"].TKCanvas, fig)
    return br
   
    ##draw_figure(window["-CANVAS2-"].TKCanvas, fig2)
    # Create an event loop
   
  
    


def update(i, n):
    subPlot.plot(i*10, n)
    subPlot.relim()
    subPlot.show()
    print("Update = {0}".format(i))
    
def openPortThread(serialQ, bootOnlyBoud, port):
    global stop_threads
    global port_connected
    global serTerminal
    stop_threads = False
    port_connected = False
    #ports = serial.tools.list_ports.comports()
    
    boudRate = 19200
    if bootOnlyBoud:
       boudRate = 230400 
    
   
    try:
        serTerminal = open_serial(port, boudRate)     
        
        t1 = threading.Thread(target = readSerial, args=[serTerminal, serialQ])
        
        t1.start()
        port_connected = True
    except:
        print("Com Port didn't find!")
    
 
# This function is called periodically from FuncAnimation
def animate(value, xs, ys):
    global subPlot
    global xIdx
    # Read temperature (Celsius) from TMP
    

    # Add x and y to lists
    xIdx = xIdx + 1
    xs.append(xIdx)
    ys.append(value)

    # Limit x and y lists to 20 items
    xs = xs[-10:]
    ys = ys[-10:]

    # Draw x and y lists
    subPlot.clear()
    subPlot.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')
    #print("PLOTING")

def updateThread(filePathName, win, bootMode):  
    print("Start Fw Update process...")
    if bootMode:
        switchToFwUpdate()
        
    getBuild() 
    switchToSlaveMode()
    fwUploadTask(False, filePathName, win)
          
def updateValues(win):
    global fwBuild
    #dataMap["level1"]
    if dataMap["TempError"] == 1:
       win.FindElement("-tempStatus-").Update("Error")
    else:
      win.FindElement("-tempStatus-").Update("OK")
    
    if dataMap["Photo"] == 1:
       win.FindElement("-photo-").Update("OPEN")
    else:
      win.FindElement("-photo-").Update("CLOSE")  
            
    win.FindElement("-temp-").Update("{0}".format(dataMap["Temp"]))
    win.FindElement("-cap1-").Update("{0}".format(dataMap["Cap1"]))
    win.FindElement("-cap2-").Update("{0}".format(dataMap["Cap2"]))
    win.FindElement("-build-").Update(fwBuild)
    
    
    
def main():

    global isSwitchedToFota
    global noPacketTimeout
    global frameErrorCounter
    global stop_threads 
    global serialQ
    global serTerminal
    global bootForce
    global boudRate
    global test_mode
    global fig
    global subPlot
    global port_connected
    global xIdx
    global dataMap
    global progress
    global fwVersion
    global fwBuild
    
    fwVersion = "____________"
    fwBuild = "_____________________"
    progress=0
    dataMap = {"level1":0, "level2": 0, "Cap1":0, "Cap2":0, "Temp":0, "TempError":0, "Photo":0}
   
    win = windows_ini()
    br = plot_ini(win)
    bootForce = False
    test_mode = False
    boudRate = RS485_BOUD_RATE_APP
    n = len(sys.argv)
    for i in range(1, n):
        print(sys.argv[i], end = " \r\n")
        if sys.argv[i] == "boot":
            bootForce = True
            boudRate = RS485_BOUD_RATE_BOOT_ONLY
            
        if sys.argv[i] == "test-mode":
            test_mode = True
            
        if sys.argv[i] == "no-force":
            boudRate = RS485_BOUD_RATE_BOOT_ONLY     
            
    serialQ = Queue() 
    test_mode = True
    print("EFLS Sensor Test environment {0}".format(version))
    print("Platform {0}".format(platform.system()))
    
    developMode = False
    
    if  developMode:
       
       getVersionFromFileName()
       
       time.sleep(1)  
       
    ports = serial.tools.list_ports.comports() 
    print([port.device for port in ports])
    win.find_element("port-list").update(values = [port.device for port in ports])
    #else:
      
           
       #t1 = threading.Thread(target = openPortThread, args=[serialQ])
       #t1.start()  
       
       
       #if not test_mode:
           
        #   if bootForce:
               #switchToFwUpdate()
           
           #getBuild() 
           #switchToSlaveMode()
           #fwUploadTask(False)
        
           
    
    i = 0
    n = 10
    
    xs = []
    ys = []
    xIdx = 0
    #ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.show()
    #win.FindElement("UPDATE").Update(disabled=True)
    getBuildISAllowed = True
    while True:
        event, values = win.read(timeout = 0.2)
        win.FindElement('fwUpdateProgress').UpdateBar(progress, 100)
        win.FindElement('progress').Update("{0}%".format(progress))
        if getDataFromSensor(win):
            animate(dataMap["level1"], xs, ys)
            if getBuildISAllowed:
                getBuild()
                getBuildISAllowed = False 
            
        updateValues(win)    
            
        # End program if user closes window or
        # presses the OK button
        #FuncAnimation(fig, update(i,n), interval=1) 
        if event == sg.WIN_CLOSED or event=="Exit":
            break
        elif event == "UPDATE":
            print("UPDATE: {0}".format(values["-FILE-"]))
            progress=0
            t2 = threading.Thread(target = updateThread, args=[values["-FILE-"], win, values["boot-checkbox"]])
            t2.start() 
        elif event == "CONNECT":
            #if win["boot-checkbox"].ch
            openPortThread(serialQ, values["boot-checkbox"], values["port-list"])
            print("CheckBox status: {0}".format(values["boot-checkbox"]))
            print("Com Port: {0}".format(values["port-list"]))
    
        
            
            
        #br = draw_figure(window["-CANVAS-"].TKCanvas, fig)
      #  a.show()
        br.draw()
       # br.draw()
        #plt.pause(5)
        i = i +1
        
        if i > 100:
            i=0
            n = n +5
        
        #if event == "OK" or event == sg.WIN_CLOSED:
         #   break

    win.close()        
    

       
       
            
def  exit_gracefully():
    sys.exit()
    raise(SystemExit) 
        
if __name__ == '__main__':
    main()
 
         
             