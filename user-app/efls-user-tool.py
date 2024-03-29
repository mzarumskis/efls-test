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
import time
import struct
from numpy.lib.function_base import rot90
from PIL.ImageEnhance import Color
from PySimpleGUI.PySimpleGUI import theme_background_color
from numpy.ma.bench import xs
from _ctypes import alignment
#from builtins import False

#from Tkinter import *

version = "V-0.0.1"

PERIODIC_DATA_RESPONCE = 1
EFLS_PROTOCOL_DEVICE_ID = 0xFE
DEBUG_DATA_RESPONCE = 15
EFLSD_PROTOCOL_CMD_OFFSET = 127


FOTA_BLOCK_SZIE_IN_BYTES = 512

DELAY_AFTER_RECEIVE = 0.05
LOOP_IN_MS = 10
NO_RECEIVE_TIMOUT_IN_S = 2 * LOOP_IN_MS

isSwitchedToFota = 0
noPacketTimeout = 0
frameErrorCounter = 0 
RS485_BOUD_RATE_BOOT_ONLY = 230400
RS485_BOUD_RATE_APP = 115200

DEFAULT_PORT = 'COM5'

#Command definitions

ZERO_CALIBRATE_CMD = 147
FULL_CALIBRATE_CMD = 148

WRITE_FOTA_RECORD = EFLSD_PROTOCOL_CMD_OFFSET + 4
WRITE_FOTA_HEADER = EFLSD_PROTOCOL_CMD_OFFSET+ 11

comunicationType = {"MASTER":0,"SLAVE":1}

 
hwModeType ={
            "UNKNOWN" :0,
            "DIGITAL_30V_RS485":1,
             "DIGITAL_5V_RS485":2,
             "DIGITAL_30V_RS232":3,
             "UNKNOWN" :4,
             "UNKNOWN" :5,
             "UNKNOWN" :6,
             "DIGITAL_5V_RS232":7,
             
             }


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



def AddCrc8(data):
  crc = 0x00
  for n in data[0:]:  
      
      for bitnumber in range(0,8):
        if ((crc ^ n) & 0x01) : 
            crc =  ((crc  ^  0x18) >> 1) | 0x80
        else:
            crc = crc >> 1
        n =  n >> 1 
  
  return crc & 0xFF

def sendToTerminal(buff):
    try:
        serTerminal.write(buff)
    except:
        dummy="error"       


def frame_BOOT_Commanded():
    global serTerminal
    buff = bytearray(b'\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E\x2E')
   
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)
     

def frame_14():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET+14)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)   

def frame_15():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET + 15)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)           
# switch to SLAVE mode
def frame_02():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET+2)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)    

# get Build    
def frame_09():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET+9)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)

def frame_04_writeRecordToFlash(recordIdx,size ,data):
    buff = bytearray(b'\x31\xFE')
    buff.append(WRITE_FOTA_RECORD)
    
    buff.append((size+4)&0xFF)
    buff.append(((size+4)>>8)&0xFF)
    
    buff.append(recordIdx & 0xFF)
    buff.append((recordIdx >> 8) & 0xFF)
    buff.append((size)&0xFF)
    buff.append((size>>8)&0xFF)
    
    for b in data:
        buff.append(b)
        
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)   

def frame_11_writeFotaHeader(version):
    buff = bytearray(b'\x31\xFE')
    buff.append(WRITE_FOTA_HEADER)
    buff.append(3)
    buff.append(0)
    buff.append(version[0])
    buff.append(version[1])
    buff.append(version[2])
              
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)   

# get config
def frame_06():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET + 6)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    try:
        serTerminal.write(buff)
    except:
        dummy="error"        

def frame_08(data):
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET + 8)
    buff.append(12)
    buff.append(0)
    
    for i in range(0,12):
        buff.append(data[i])
    
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)    

def frame_19(data):
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET + 19)
    buff.append(28)
    buff.append(0)
    
    for i in range(0,28):
        buff.append(data[i])
    
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff) 
    
    
def frame_calibrate(cmd):
    buff = bytearray(b'\x31\xFE')
    buff.append(cmd)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)    
    
    
def LLS_frame_19(intervalT):
    buff = bytearray(b'\x31')
    buff.append(dataMap["DeviceID-TH"])
    buff.append(0x13)
    buff.append(intervalT)
    crc = AddCrc8(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)         
    
        
    
def getDataFromSerial():
    data = serialQ.get()
    if data:
        #time.sleep(1/LOOP_IN_MS) #delay 50mS 
        return data
    
    return ""   

def getDataFromSerialWithTimeout(timeOut):
    try:
        data = serialQ.get(timeout = timeOut)
        return data
    except:
        dummy = "error"
        #print("Timeout")
       
    return ""  

# get calibration
def frame_18():
    buff = bytearray(b'\x31\xFE')
    buff.append(EFLSD_PROTOCOL_CMD_OFFSET + 18)
    buff.append(0)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    sendToTerminal(buff)   

def getAnyFromRS485(timeoutInmS = 500):
    count = timeoutInmS/100
    
    while count:
        try:
            if serialQ.get(timeout = 0.1):
                return
        except:
            dummy ="error"
                 
        time.sleep(1/LOOP_IN_MS) #delay 50mS
        count = count - 1
         
def readSerial(ser, serialQ):
    global stop_threads
    
    while (ser.isOpen()):
        if stop_threads:
            break  
        
        rawdata = ser.read(256)
        
        if rawdata:
            serialQ.put(rawdata)
           
          
        time.sleep(1/LOOP_IN_MS) #delay 50mS
        
        

def isPackedValid(data):
    
    if len(data) < 6:
        return False
    print("LEN: {0}".format( len(data)))
    print("LENGHT: {0}".format(((data[4]<<8) | data[3])+5))
    if len(data) < (((data[4]<<8) | data[3])+5):
        return False
    
    #print("LENGHT2: {0}".format(((data[4]<<8) | data[3])+3))
    if data[0] != 0x31:
        return False 
    
    return packetCrcCheck(data)

def isBuildAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 10:
        return True
    return False

def isGetConfigFrame(data):
    if data[2] ==EFLSD_PROTOCOL_CMD_OFFSET+ 7:
        return True
    return False

def isSwticToSlavedAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 3 and data[5] == EFLSD_PROTOCOL_CMD_OFFSET+2 :
        return True
    return False

def isWriteRecorddAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 5 and data[5] == EFLSD_PROTOCOL_CMD_OFFSET+ 4 :
        return True
    return False
def isWriteConfigAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 3 and data[5] ==EFLSD_PROTOCOL_CMD_OFFSET+ 8 :
        return True
    return False

def isWriteCalibrationAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 3 and data[5] == EFLSD_PROTOCOL_CMD_OFFSET+19 :
        return True
    return False

def isCalibrationAckFrame(data):
    if data[2] == EFLSD_PROTOCOL_CMD_OFFSET + 3 and (data[5] == 147 or data[5] == 148):
        return True
    return False
         
def packetCrcCheck(data):
    crc = 0x0000
    for n in data[2:data[3]+5]:  
      crc = crc ^ (n << 8) 
      for bitnumber in range(0,8):
        if crc & 0x8000 : 
            crc =  crc  ^  (0x1070 << 3)
        crc = ( crc << 1 )
    crc = crc >> 8
    if crc & 0xFF == data[data[3]+5]:
        return True
    else:
        return False

def setConfig(win, sensorLength, deviseid, sendPeriod, mode, boudRate):
    win.FindElement('configProgress').UpdateBar(0, 20)
    win.FindElement('configStatus').Update("Writing...       ")
    buff = bytearray(b'\x00\x00\x00\x00')
    print("setConfig: {0}".format(sensorLength))
    struct.pack_into('f', buff, 0,float(sensorLength))
    print("",buff[0],buff[1],buff[2],buff[3])
    
    #----- Sensor Lenght ------
    configBuf[3] = buff [3]
    configBuf[4] = buff [2]
    configBuf[5] = buff [1]
    configBuf[6] = buff [0]
    
    #Send Period
    configBuf[0] = (int(sendPeriod) >> 8)&0xFF
    configBuf[1] = int(sendPeriod)&0xFF
        
   
          
    configBuf[2] = int(deviseid)&0xFF
    
    print("Mode set: {0}".format(comunicationType[mode]))
    configBuf[7] = comunicationType[mode]
    
    struct.pack_into('i', buff, 0,int(boudRate))
    print("",buff[0],buff[1],buff[2],buff[3])
    configBuf[8] = buff[3]
    configBuf[9] = buff[2]
    configBuf[10] = buff[1]
    configBuf[11] = buff[0]
    
    if(needWaitForLss()):
        getAnyFromRS485()
        
    frame_08(configBuf)
    
    try:
        data = getDataFromSerialWithTimeout(0.5)
        print("data")
        if isPackedValid(data):
            if isWriteConfigAckFrame(data):
                print("Write OK")
                win.FindElement('configProgress').UpdateBar(20, 20)
                win.FindElement('configStatus').Update("Write OK       ")
                updateConfig(boud = boudRate)
                win.write_event_value("RE-CONNECT", "")
            
    except :
        print("Timeout")
        win.FindElement('configStatus').Update("Write ERROR  ")
 
def fromByteArrayToFloat(data):
     
      bytestoFloat = bytearray()
      bytestoFloat.append(data[3])
      bytestoFloat.append(data[2])
      bytestoFloat.append(data[1])
      bytestoFloat.append(data[0])
      tup = struct.unpack('f',bytestoFloat)
      return tup [0] 

def fromByteArrayToInt(data):
     
      bytestoInt = bytearray()
      bytestoInt.append(data[3])
      bytestoInt.append(data[2])
      bytestoInt.append(data[1])
      bytestoInt.append(data[0])
      tup = struct.unpack('i',bytestoInt)
      return tup [0] 
  
def getModeByEnum(mode):
    for modeEnum in comunicationType:
        if mode == comunicationType[modeEnum]:
            return modeEnum
    return ""    

def getHwModeByEnum(mode):
    for modeEnum in hwModeType:
        if mode == hwModeType[modeEnum]:
            return modeEnum
    return "UNKNOWN"    
    
    
def needWaitForLss():
    global lastLlsPacketTimeStamp
    
    timenow = int(round(time.time() * 1000))
    diff =  timenow - lastLlsPacketTimeStamp          
    if diff > 2700:
        return True
    else:
        return False    
        
    
def getConfig(win):
    global configBuf
    win.FindElement('configProgress').UpdateBar(0, 20)
    win.FindElement('configStatus').Update("Reading...       ")
    win.Refresh()
    
    if(needWaitForLss()):
        getAnyFromRS485()
    frame_06()
   
       
    try:
        data = getDataFromSerialWithTimeout(0.5)
    
        if isPackedValid(data):
           if isGetConfigFrame(data):
              
               print("Data->RX {0}".format(data.hex().upper()))
               for i in range(0,11):
                   configBuf[i] = data [i+5]
              
               
               dataMap["Sensor-Lenght"] = fromByteArrayToFloat(data[8:12])
               dataMap["DeviceID-TH"] = data[7]
               
               dataMap["Send-Interval"] = data[5]<<8 | data[6]
               
               dataMap["Mode"] = data[12]
               
               dataMap["boudRate"] = fromByteArrayToInt(data[13:17])
               
               print("Mode= {0}".format(dataMap["Mode"] ))
               print("Mode TXT= {0}".format(getModeByEnum(dataMap["Mode"]) ))
               
              
              
               win.FindElement('Mode').Update(getModeByEnum(dataMap["Mode"])) 
               
               win.FindElement('-boudRate-').Update(dataMap["boudRate"]) 
               
               #print("Converted= {0}".format(struct.unpack('f',b'\x00\x80\x27\x44')));
               print("Converted= {0}".format(dataMap["Sensor-Lenght"]))
               
               win.FindElement("-sensor-lenght-").Update("{:0.0f}".format(dataMap["Sensor-Lenght"] ))
               win.FindElement("-DevID-").Update("{0}".format(dataMap["DeviceID-TH"] ))
               
               win.FindElement("-Send-Interval-").Update("{0}".format(dataMap["Send-Interval"]))
              
               
              
               win.FindElement('configProgress').UpdateBar(20, 20)
               win.FindElement('configStatus').Update("Read OK")
        return True 
    except:
      
        print("Timeout")
        win.FindElement('configStatus').Update("Read ERROR  ")
        return False
    
def getCalibration(win, wait):
    global  calibrationDataBuf  
    
    if wait:
        if(needWaitForLss()):
            getAnyFromRS485()
    frame_18()
   
       
    try:
        data = getDataFromSerialWithTimeout(10)
    
        if isPackedValid(data):
           print("Data->RX {0}".format(data.hex().upper()))
           for i in range(0,28):
               calibrationDataBuf[i] = data [i+5]
           
           dataMap["param-mainCapZero"] = fromByteArrayToFloat(data[5:9])
           dataMap["param-mainCapParazitic"] = fromByteArrayToFloat(data[9:13])   
           dataMap["param-refCapParazitic"] = fromByteArrayToFloat(data[13:17])   
           dataMap["param-epsilion"] = fromByteArrayToFloat(data[17:21])
           dataMap["param-airGapInMM"] = fromByteArrayToFloat(data[21:25])  
           dataMap["param-zero-override"] = fromByteArrayToFloat(data[25:29])    
           
           dataMap["param-calibration-full"] = fromByteArrayToFloat(data[29:33])
              
           win.FindElement("PARAM-MAIN_CAP_PARAZITIC").Update("{:0.2f}".format(dataMap["param-mainCapParazitic"] ))
           win.FindElement("PARAM-REF_CAP_PARAZITIC").Update("{:0.2f}".format(dataMap["param-refCapParazitic"] ))
           win.FindElement("PARAM-MAIN_CAP_ZERO").Update("{:0.2f}".format(dataMap["param-mainCapZero"] ))  
           win.FindElement("PARAM-EPSILION").Update("{:0.3f}".format(dataMap["param-epsilion"] ))
           win.FindElement("PARAM-AIR_GAP_PLATE").Update("{:0.2f}".format(dataMap["param-airGapInMM"] ))  
           win.FindElement("PARAM-MAIN_CAP_ZERO_OVERRIDED").Update("{:0.2f}".format(dataMap["param-zero-override"] ))  
           
           win.FindElement("-calibrationZero-").Update("{:0.2f}".format(dataMap["param-zero-override"] ))  
           
           win.FindElement("-calibrationFull-").Update("{:0.2f}".format(dataMap["param-calibration-full"] ))  
           
           print("Read OK")
           win.FindElement('ParamSetStatus').Update("Read OK       ")
        return True 
    except:
      
        print("Timeout")
       
        return False
    
def setCalibrationData(win, mainCapZeroOffset, mainCapParazitic, refCapParazitic, epsilion,airGap, mainCapZeroOwerrided):
    global  calibrationDataBuf  
    buff = bytearray(b'\x00\x00\x00\x00')
    win.FindElement('ParamSetStatus').Update("               ")
    win.Refresh()
    struct.pack_into('f', buff, 0,float(mainCapZeroOffset))
    
    #----- mainCapZeroOffset ------
    calibrationDataBuf[0] = buff [3]
    calibrationDataBuf[1] = buff [2]
    calibrationDataBuf[2] = buff [1]
    calibrationDataBuf[3] = buff [0]
      
    struct.pack_into('f', buff, 0,float(mainCapParazitic))
    calibrationDataBuf[4] = buff[3]
    calibrationDataBuf[5] = buff[2]
    calibrationDataBuf[6] = buff[1]
    calibrationDataBuf[7] = buff[0]
    
    struct.pack_into('f', buff, 0,float(refCapParazitic))
    calibrationDataBuf[8] = buff[3]
    calibrationDataBuf[9] = buff[2]
    calibrationDataBuf[10] = buff[1]
    calibrationDataBuf[11] = buff[0]
    
    
    struct.pack_into('f', buff, 0,float(epsilion))
    calibrationDataBuf[12] = buff[3]
    calibrationDataBuf[13] = buff[2]
    calibrationDataBuf[14] = buff[1]
    calibrationDataBuf[15] = buff[0]
    
    struct.pack_into('f', buff, 0,float(airGap))
    calibrationDataBuf[16] = buff[3]
    calibrationDataBuf[17] = buff[2]
    calibrationDataBuf[18] = buff[1]
    calibrationDataBuf[19] = buff[0]
    
    struct.pack_into('f', buff, 0,float(mainCapZeroOwerrided))
    calibrationDataBuf[20] = buff[3]
    calibrationDataBuf[21] = buff[2]
    calibrationDataBuf[22] = buff[1]
    calibrationDataBuf[23] = buff[0]
        
        
    if(needWaitForLss()):
        getAnyFromRS485()
        
    frame_19(calibrationDataBuf)
    
    try:
        data = getDataFromSerialWithTimeout(0.5)
        print("data")
        if isPackedValid(data):
            if isWriteCalibrationAckFrame(data):
                print("Write OK")
                win.FindElement('ParamSetStatus').Update("Write OK       ")
            
    except :
        print("Timeout")
        win.FindElement('ParamSetStatus').Update("Write ERROR  ")    

def calibrateSensor(win,cmd):
    
    if cmd == ZERO_CALIBRATE_CMD:
       statusReport = 'calibrationStatus'
    else:
       statusReport = 'calibrationStatus'
                    
    if(needWaitForLss()):
        getAnyFromRS485()     
    frame_calibrate(cmd)
    
    try:
        data = getDataFromSerialWithTimeout(0.5)
        print("Data->RX {0}".format(data.hex().upper()))
        if isPackedValid(data):
            if isCalibrationAckFrame(data):
                print("Calibrate OK")
                
                win.FindElement(statusReport).Update("Calibrate DONE!   ")
                    
                return True
        return  False    
    except :
        print("Calibration ACK not received")
        
        win.FindElement(statusReport).Update("Calibrate ERROR!   ")
        
    return False    
def switchToFwUpdate():
    global updateTaskTerminat
    dummy =""
    while True:
        frame_BOOT_Commanded()
        try:
            data = getDataFromSerialWithTimeout(0.3)
            print("Data->RX {0}".format(data.hex().upper()))
            if isPackedValid(data):
               return True 
        except :
            dummy="e"
            print("Timeout")
            if updateTaskTerminat==True:
                return False
               
   # data = getDataFromSerial()

def getBuild():
    global fwBuild
    global updateTaskTerminat
    tryCount = 3
    while(tryCount):
        if updateTaskTerminat == True:
            return False
        #getAnyFromRS485()
        #time.sleep(DELAY_AFTER_RECEIVE) #delay 
        frame_09()
        try:
            data = getDataFromSerialWithTimeout(0.5)
            if data:
                
                print("Data->RX {0}".format(data.hex().upper()))
              
              
                tryCount = tryCount - 1
                if isPackedValid(data):
                     if isBuildAckFrame(data):
                         buildDecode = data[5:((data[4]<<8) | data[3])+4]
                         print("Build {:s}".format(buildDecode.decode('utf-8')))
                         fwBuild = "{0}".format(buildDecode.decode('utf-8'))
                         return True
                else:
                    print("Packet ERROR")
            else:
                print("NO DATA")
                tryCount = tryCount - 1
                #break  
        except:
            tryCount = tryCount - 1
            #break          
    return False            

def getDataFromSlave():
    frame_15()

def switchToSlaveMode():
    global updateTaskTerminat
    tryCount = 5
    #getAnyFromRS485()
    while(tryCount):
        if updateTaskTerminat == True:
            return False
        #time.sleep(DELAY_AFTER_RECEIVE) #delay 
        frame_02()
        try:
            data = getDataFromSerialWithTimeout(1.5)
            print("Data->RX {0}".format(data.hex().upper()))
            tryCount = tryCount - 1
            if isPackedValid(data):
                 if isSwticToSlavedAckFrame(data):
                     print("Slave Mode Entered")
                     return True
            else:
                print("Packet ERROR")
                tryCount = tryCount - 1
        except :
           tryCount = tryCount - 1 
           print("Timeout")    
    return False       
            
def waitWriteRecordAck(idx):
    try:
        data = getDataFromSerialWithTimeout(2)
        print("Data->RX {0}".format(data.hex().upper()))
        if isPackedValid(data):
           if isWriteRecorddAckFrame(data):
               idxAck = (data[7]<<8) + data[6]
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
    global updateTaskTerminat
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
                    if updateTaskTerminat:
                        progress=0 
                        break
                    data = file.read(FOTA_BLOCK_SZIE_IN_BYTES)
                    if data:
                        writeOk = False
                        
                        while not writeOk:
                            if updateTaskTerminat:
                                progress=0 
                                break    
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
    global fwVersion
    global lastLlsPacketTimeStamp
    dummy =""
   
    try:
        data = getDataFromSerialWithTimeout(0.5)
        #print("Data LLS->RX {0}".format(data.hex().upper()))  
        print("Data->RX {0}".format(data.hex().upper()))
              ##return True
        if isPackedValid(data):
           #print("Data->RX {0}".format(data.hex().upper())) 
           if data [2] == PERIODIC_DATA_RESPONCE + EFLSD_PROTOCOL_CMD_OFFSET:
               updateCapacityAndLevelInfo(data)
           elif data [2] == DEBUG_DATA_RESPONCE + EFLSD_PROTOCOL_CMD_OFFSET:
               updateCapacityAndLevelInfo(data)                 
               print("Min CAP: {0};".format( data[21]<<8 |data[22]))
               print ("DATA")
               print("Tem Sensors: {0}, {1}, {2};".format(convertToSignChar(data[23]), convertToSignChar(data[24]), convertToSignChar(data[25])))
               return True
        else:
            if data [1] != EFLS_PROTOCOL_DEVICE_ID:
                print("Data LLS->RX {0}".format(data.hex().upper()))
                lastLlsPacketTimeStamp = int(round(time.time() * 1000))
                #win.write_event_value("getSlaveData", "")
        return False
    except :
        dummy="e" 
 
        
        return False    

def getLLSPacket(win):
    global fwVersion
    global lastLlsPacketTimeStamp
    dummy =""
   
    try:
        data = getDataFromSerialWithTimeout(0.1)
        #print("Data LLS->RX {0}".format(data.hex().upper()))  
        print("Data->RX {0}".format(data.hex().upper()))
              ##return True
        if isPackedValid(data):
           #print("Data->RX {0}".format(data.hex().upper())) 
           
               return False
        else:
            if data [1] != EFLS_PROTOCOL_DEVICE_ID:
                print("Data LLS->RX {0}".format(data.hex().upper()))
                lastLlsPacketTimeStamp = int(round(time.time() * 1000))
                return True
                #win.write_event_value("getSlaveData", "")
        return False
    except :
        dummy="e" 
 
        
        return False    
def convertToSignChar(data):
    if data & 0x80:
        return (256-data) * (-1)
    else:
        return data        
    
def updateCapacityAndLevelInfo (data):
    global fwVersion
    dataMap["Cap1"] = data[8] << 8 | data[9]
    dataMap["level1"] = data[14]<<8 | data[15]
    fwVersion = "V{:02X}.{:02X}.{:02X}".format(data [18],data[19], data[20])
    
    dataMap["hwVersion"] = data [23]
    dataMap["errorFlag"] = data [24]
    print("CAP1: {0}pF; Level1: {1}; HW: {2} {3} Er: {4}".format( data[8]<<8 |data[9], dataMap["level1"], getHwModeByEnum(dataMap["hwVersion"]), dataMap["hwVersion"], dataMap["errorFlag"] ))

               

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return figure_canvas_agg                        

def windows_ini(width, high):
    matplotlib.use("TkAgg")
    frame2_layout = [
        [sg.Text("                                                          ")],
        [sg.Text("Choose a file: "), sg.FileBrowse("File", key="-FILE-")],
        [sg.Button("UPDATE"), sg.Button("STOP")],
        [sg.Button("RESET")],
        [sg.ProgressBar(orientation="horizontal",max_value=10,size=(10, 20), key = "fwUpdateProgress"), sg.Text("0%  ", key = "progress")],
          [sg.Text("Level MinMax:"), sg.Text("                 ", key = "minMax")],
        ]
    information_column = [
                            [sg.Checkbox("Boot Only", key = "boot-checkbox", visible=False), sg.Combo("",size =(10,1), key = "port-list")],
                            [sg.Button("CONNECT"), sg.Button("RE-CONNECT", visible=False)],
                            [sg.Text("Fw Version:"), sg.Text("____________", key="-version-") ],
                            [sg.Text("Build:"), sg.Text("__________________________________", key = "-build-")],
                            [sg.Text("HW:"), sg.Text("__________________________________", key = "-hwVerTXT-")],
                            [sg.HSeparator(color = "White"),],
                            [sg.Text("                                                          ")],
                            [sg.Text("Temp:", visible=False),sg.Text("####", key="-temp-",visible=False) ],
                            [sg.Text("Status",visible=False),sg.Text("####", size = (6,1),key="-tempStatus-", visible=False) ],
                            [sg.Text("")],
                            [sg.Text("#######", key = "-zeroCalibrationStatus-", size =(28,1),auto_size_text =True)],
                            [sg.Button("CALIBRATE ZERO", key = "CALIBRATE-ZERO"), sg.Text("####", size = (6,1),key="-calibrationZero-"), sg.Text("pF")],
                            [sg.Text("")],
                            [sg.Text("#######", key = "-fullCalibrationStatus-", size =(28,1),auto_size_text =True)],
                            [sg.Button("CALIBRATE FULL", key = "CALIBRATE-FULL"),  sg.Text("####", size = (6,1),key="-calibrationFull-"), sg.Text("pF")],
                            [sg.Text("FLASH:", visible=False),sg.Text("####", key="-errorflg-", visible=False)] ,
                            [sg.Text("CAP2:",visible=False),sg.Text("####", key="-cap2-",visible=False) ,sg.Text("pF",visible=False)],
                            [sg.Text("")],
                            [sg.Text("Photo:",visible=False),sg.Text("ON", size = (6,1), key="-photo-", visible=False) ],
                            [sg.Button("READ", key = "getSlaveData", visible=False)],                           
                            [sg.Text("")],
                            [sg.Frame(layout = frame2_layout,  title="FW", visible=False)],
                         ]
   
    
    information_column2_1 = [
                            
                            [sg.ProgressBar(orientation="vertical",max_value=100,size=(14, 25), key ="levelBar1", pad = ((0,0),(70,0))) ],
                           
                           
                            
                           
                            
                           
                            ]
    
    information_column2_2 = [
                            [sg.Text("             ", key="Bar1Value")],
                            [sg.Image("EFLS-sensor-nobkgr100x199.png", size=(70,199), pad = ((0,30),(0,0)))],
                           
                          
                           
                           
                            ]
     
    information_column2_3 = [
                            
                            [sg.Canvas(key="-CANVAS-")],
                           
                            [sg.Text("Capacity:"),sg.Text("####", key="-cap1-") ,sg.Text("pF")],                    
                            
                            
                           
                            ]
    
    tab_layout_info = [
        [sg.Column(information_column2_1,  size=(width-1000, high-368)),
         sg.Column(information_column2_2, size=(width-950, high-368)),
         sg.Column(information_column2_3, size=(width-450, high-348))],
        
        
        ]
    tab_layout_config_param_list = [
      
        [sg.Text("Sensor lenght: ", visible=False)],
        [sg.Text("Device ID:  ")],
        [sg.Text("Send Interval:  ")],
        [sg.Text("MODE AT START:  ")],
        [sg.Text("RS485 Boud: ")],
        ]
    
    tab_layout_config_param_input = [
      
        [sg.Input(size=(5, 1),  key = "-sensor-lenght-", visible=False),sg.Text("mm", visible=False)],
        [sg.Input(size=(5, 1),  key = "-DevID-")],
        [sg.Input(size=(5, 1),  key = "-Send-Interval-"),sg.Text("s")],
        [sg.Combo(["MASTER", "SLAVE"],size =(10,1), key = "Mode")],
        [sg.Input(size=(5, 1),  key = "-boudRate-")],
         ]
    
    
    tab_layout_config=[
         [sg.Column(tab_layout_config_param_list ), sg.Column(tab_layout_config_param_input),],
         [sg.Text("Status:"), sg.Text("                 ", key = "configStatus")],
         [sg.ProgressBar(max_value=10,size=(20, 5), key="configProgress")],
         [sg.Text("  ")],
         [sg.Button("READ", key = "-read-button-"), sg.Button("WRITE", key = "-write-button-")],
         [sg.Text("  ")],
         [sg.Button("LSS SET INTERVAL", key = "button-lss-set", visible=False)],
        ]
    
    tab_layout_param_list=[
        
         [sg.Text("MAIN CAP ZERO OFFSET: ")],
         [sg.Text("MAIN CAP ZERO VALUE: ")],
         [sg.Text("MAIN CAP PARAZITIC: ")],
         [sg.Text("REFERENCE CAP PARAZITIC: ")],
         [sg.Text("PlATE AIR GAP: ")],
         [sg.Text("EPSILION: ")],
        
        ]
    tab_layout_param_input=[
        
         [sg.Input(size=(6, 1),  key = "PARAM-MAIN_CAP_ZERO"),sg.Text("pF")],
         [sg.Input(size=(6, 1),  key = "PARAM-MAIN_CAP_ZERO_OVERRIDED"),sg.Text("pF")],
         [sg.Input(size=(6, 1),  key = "PARAM-MAIN_CAP_PARAZITIC"),sg.Text("pF")],
         [sg.Input(size=(6, 1),  key = "PARAM-REF_CAP_PARAZITIC"),sg.Text("pF")],
         [sg.Input(size=(6, 1),  key = "PARAM-AIR_GAP_PLATE"),sg.Text("mm")],
         [sg.Input(size=(6, 1),  key = "PARAM-EPSILION")],
        
        ]
    
    tab_layout_parameters =[
        [sg.Column(tab_layout_param_list ), sg.Column(tab_layout_param_input ),],
        [sg.Text("Status:"), sg.Text("                 ", key = "ParamSetStatus")],
        [sg.Button("READ", key = "PARAMETERS_READ"), sg.Button("WRITE", key = "PARAMETERS_WRITE")],
        [sg.Text("")],
        
        [sg.Text("Status:"), sg.Text("                 ", key = "calibrationStatus")],
         ]
    
    tab_control = [
                    [sg.Tab("Info", layout= tab_layout_info)],
                    [sg.Tab("Config", layout= tab_layout_config, visible=True)],
                    [sg.Tab("Parameters", layout= tab_layout_parameters, visible=False)]
                  ]
    
    infeomation_col = [[sg.Frame(layout = information_column, title="Main",  vertical_alignment = "top", key= "frame1")]]
    
    layout = [ 
                [sg.Column(infeomation_col,vertical_alignment = "top" ), sg.Column([[sg.TabGroup(tab_control)]],size =(width-300, high-308), vertical_alignment = "bottom")] ,
              
               
             ]
   
 
    
    

    #x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
    #y= np.array ([0,10,20,30,40,50,60,70,80,90,100])
    print("Starting")
    
     # Create the window
    window = sg.Window("EFLS-D2 Test " + version, layout, size=(width, high - 280),finalize=True,
    element_justification="left",
    font="Helvetica 12",
    
    )
    return window
    
def plot_ini(window):
    global fig
    global subPlot
    fig = Figure(figsize=(5.0,3.6))
    subPlot = fig.add_subplot(111)
    subPlot.axis([0, 10, 0, 100])
    
    #a.scatter(100,10,color='red')
    #a.plot(p, range(2 +max(x)),color='blue')
    #a.invert_yaxis()

    subPlot.set_title ("Level", fontsize=10)
    subPlot.set_ylabel("%", fontsize=8, rotation=180)
    subPlot.set_xlabel("t", fontsize=8)
    
     
    br = draw_figure(window["-CANVAS-"].TKCanvas, fig)
    return br

    


def update(i, n):
    subPlot.plot(i*10, n)
    subPlot.relim()
    subPlot.show()
    ##print("Update = {0}".format(i))
    
def openPortThread(serialQ, bootOnlyBoud, port):
    global stop_threads
    global port_connected
    global serTerminal
    stop_threads = False
    port_connected = False
    #ports = serial.tools.list_ports.comports()
    
    boudRate = RS485_BOUD_RATE_APP
    if bootOnlyBoud:
       boudRate = RS485_BOUD_RATE_BOOT_ONLY 
    else:
        if getDefaultBoudRateFromConfig() != 0:
           boudRate = getDefaultBoudRateFromConfig()
           print("Boud updated")
    
    print("COM: {0} {1}".format(port,boudRate))
    
    try:
        serTerminal = open_serial(port, boudRate)     
        
        t1 = threading.Thread(target = readSerial, args=[serTerminal, serialQ])
        updateConfig(port)
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
    print(xs)
    
    if len(xs)< 10:
        x_move = 10
    else:
        x_move = xs[9]       
            
    ##print(x_move)
    # Draw x and y lists
    subPlot.clear()
    subPlot.plot(xs, ys)
    
    
    subPlot.axis([xs[0],x_move, 0, 100])

    subPlot.set_title ("Level", fontsize=10)
    subPlot.set_ylabel("%", fontsize=8)
    subPlot.set_xlabel("t", fontsize=8)
    
    # Format plot
    #plt.xticks(rotation=45, ha='right')
    #plt.subplots_adjust(bottom=0.30)
    #plt.title('TMP102 Temperature over Time')
    #plt.ylabel('Temperature (deg C)')

def animate2(value, xs, ys):
    global subPlot2
    global xIdx2
    # Read temperature (Celsius) from TMP
    

    # Add x and y to lists
    xIdx2 = xIdx2 + 1
    xs.append(xIdx2)
    ys.append(value)

    # Limit x and y lists to 20 items
    xs = xs[-10:]
    ys = ys[-10:]
    
    if len(xs)< 10:
        x_move = 10
    else:
        x_move = xs[9]     
    # Draw x and y lists
    subPlot2.clear()
    subPlot2.plot(xs, ys)
    
    subPlot2.axis([xs[0],x_move, 0, 100])
    subPlot2.set_title ("CAP2", fontsize=10)
    subPlot2.set_ylabel("Y", fontsize=8)
    subPlot2.set_xlabel("t", fontsize=8)
    # Format plot
    #plt.xticks(rotation=45, ha='right')
    #plt.subplots_adjust(bottom=0.30)
    #plt.title('TMP102 Temperature over Time')
    #plt.ylabel('Temperature (deg C)')    
    
    
    #print("PLOTING")



def updateThread(filePathName, win, bootMode):  
    global updateInProgress
    global updateTaskTerminat
    print("Start Fw Update process...")
    if bootMode:
        if switchToFwUpdate() ==False:
            return
    if getBuild() == False:
        return 
    if switchToSlaveMode() == False:
        return
    fwUploadTask(False, filePathName, win)
    updateInProgress = False
    print("END Update process...")    
          
def checkFlashError(errorFlag):
    if(errorFlag & 0x01 !=0):
        return "ERROR"
    else:
        return "OK"
              
          
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
    win.FindElement("-version-").Update(fwVersion)
    win.FindElement("-hwVerTXT-").Update(getHwModeByEnum(dataMap["hwVersion"]))
    win.FindElement("-errorflg-").Update(checkFlashError(dataMap["errorFlag"]))
    
    
    
    
    

MIN_MAX_DETECTION_LEVEL = 87.0     

def updateConfig (port = 'none', boud = 0):
    config = ConfigParser()   
    config.read('config.ini')
    
    if port != 'none':
        config.set('main', 'port', port)
    if boud != 0:
        config.set('main', 'boud', boud)
    
    with open('config.ini', 'w') as f:
        config.write(f)
    
    print("setting Port: "+config.get('main', 'port'))
    print("setting boud: "+config.get('main', 'boud'))

    
def getDefaultBoudRateFromConfig():
    boud = 0
    config = ConfigParser()   
    config.read('config.ini')
    
    try :
        if config.get('main', 'boud') !='':
           boud = config.get('main', 'boud')
    except:
        print("")
         
    return boud

from configparser import ConfigParser    


def mainReadFromSensor(win):
    global updateInProgress
    global minMaxEnables
    global dataMap
    global minmax
    global xs,ys,xs2,ys2
    global getBuildISAllowed
    
    if not updateInProgress:
        if getLLSPacket(win):
            
           getDataFromSlave()
           getDataFromSensor(win)  
           getCalibration(win, False) 
           
           animate(dataMap["level1"]/10.0, xs, ys)
            
            
           win.FindElement('levelBar1').UpdateBar(dataMap["level1"]/10.0, 100)
           win.FindElement('Bar1Value').Update("{0}%".format(dataMap["level1"]/10.0))
           if getBuildISAllowed:
                if getBuild():
                    getBuildISAllowed = False 
            
        updateValues(win)    

    
def main():

#360
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
    global fig2
    global subPlot2
    global port_connected
    global xIdx
    global xIdx2
    global dataMap
    global progress
    global fwVersion
    global fwBuild
    global updateInProgress
    global configBuf
    global updateTaskTerminat
    global minmax
    global minMaxEnables 
    global calibrationDataBuf
    global lastLlsPacketTimeStamp
    global xs,ys,xs2,ys2
    global getBuildISAllowed
       
    lastLlsPacketTimeStamp = 0   
    config = ConfigParser()   
    config.read('config.ini')
    
    #config.set('main', 'port', '')
    #with open('config.ini', 'rw') as f:
    #    config.write(f)
        
    minmax = [MIN_MAX_DETECTION_LEVEL,MIN_MAX_DETECTION_LEVEL]
    updateTaskTerminat = False
    minMaxEnables = False
    configBuf = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00') 
    calibrationDataBuf = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00') 
    updateInProgress= False
    fwVersion = "____________"
    fwBuild = "_____________________"
    progress=0
    dataMap = {"level1":0, "level2": 0, "Cap1":0, "Cap2":0, "Temp":0, "TempError":0, "Photo":0, "Sensor-Lenght":0.0, "DeviceID-TH":0, "Send-Interval":0, "Mode":-1, "fwVersion":"", "boudRate":0, 
               "param-mainCapZero":0.0,
               "param-mainCapParazitic":0.0,
               "param-refCapParazitic":0.0,
               "param-epsilion":0.0,
               "param-airGapInMM":0.0,
               "param-zero-override":0.0,
               "param-calibration-full":0.0,
               "hwVersion":0,
               "errorFlag":0,
               }
   
    win = windows_ini(1024,768)
    br = plot_ini(win)
    
    
    bootForce = False
    test_mode = False
    boudRate = RS485_BOUD_RATE_APP
    getDataRequest = False
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
    win.find_element("port-list").update(values = [port.device for port in ports])
    #win.find_element("port-list").update(values = [DEFAULT_PORT])
    print([port.device for port in ports])
    
    
    try :
        if config.get('main', 'port') !='':
           print("setting: "+config.get('main', 'port')) 
           win.find_element("port-list").update( set_to_index=[config.get('main', 'port')])
           win.find_element("port-list").update( value=[config.get('main', 'port')])
    except:
        print("")
         
    
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
    
    xs2 = []
    ys2 = []
    xIdx2 = 0
    #ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.show()
    #win.FindElement("UPDATE").Update(disabled=True)
    getBuildISAllowed = True
    while True:
        event, values = win.read(timeout = 0.01)
        #win.FindElement('fwUpdateProgress').UpdateBar(progress, 100)
        #win.FindElement('progress').Update("{0}%".format(progress))
        
        mainReadFromSensor(win)
                
        # End program if user closes window or
        # presses the OK button
        #FuncAnimation(fig, update(i,n), interval=1) 
        if event == sg.WIN_CLOSED or event=="Exit":
            break
        elif event == "UPDATE":
            print("UPDATE: {0}".format(values["-FILE-"]))
            progress=0
            updateInProgress = True
            updateTaskTerminat = False
            t2 = threading.Thread(target = updateThread, args=[values["-FILE-"], win, values["boot-checkbox"]])
            t2.start() 
        elif event == "CONNECT":
            #if win["boot-checkbox"].ch
            openPortThread(serialQ, values["boot-checkbox"], values["port-list"])
            print("CheckBox status: {0}".format(values["boot-checkbox"]))
            print("Com Port: {0}".format(values["port-list"]))
        elif event == "-read-button-":
            print("Read Setting")
            getConfig(win)
        elif event == "-write-button-":
            print("Write Setting")
            setConfig(win, values["-sensor-lenght-"], values ["-DevID-"], values["-Send-Interval-"], values["Mode"], values["-boudRate-"])
        elif event == "STOP":  
            updateTaskTerminat = True  
            progress=0
        elif event == "RE-CONNECT":
            stop_threads = True
            updateTaskTerminat = True
            time.sleep(0.3) #delay 
            serTerminal.close()  
            port_connected = False
            stop_threads = False
            updateTaskTerminat = False
            openPortThread(serialQ, values["boot-checkbox"], values["port-list"])
            print("CheckBox status: {0}".format(values["boot-checkbox"]))
            print("Com Port: {0}".format(values["port-list"]))  
        elif event == "getSlaveData":
            getDataFromSlave()
            mainReadFromSensor(win)
        elif event == "RESET":
            minmax = [MIN_MAX_DETECTION_LEVEL,MIN_MAX_DETECTION_LEVEL]
            minMaxEnables = False
        elif event == "PARAMETERS_READ":
             getCalibration(win, True) 
        elif event == "PARAMETERS_WRITE":
             setCalibrationData(win, values["PARAM-MAIN_CAP_ZERO"], 
                                     values["PARAM-MAIN_CAP_PARAZITIC"], 
                                     values["PARAM-REF_CAP_PARAZITIC"], 
                                     values["PARAM-EPSILION"],
                                     values["PARAM-AIR_GAP_PLATE"],
                                     values["PARAM-MAIN_CAP_ZERO_OVERRIDED"],
                                     ) 
    
        elif event == "CALIBRATE-ZERO":  
            win.FindElement('-zeroCalibrationStatus-').Update("Calibrating..") 
              
            if calibrateSensor(win, ZERO_CALIBRATE_CMD):
                win.FindElement('-zeroCalibrationStatus-').Update("Calibration OK!")  
            else:
                win.FindElement('-zeroCalibrationStatus-').Update("Error try again!")  
        elif event == "CALIBRATE-FULL": 
            win.FindElement('-fullCalibrationStatus-').Update("Calibrating..")     
            
            if calibrateSensor(win, FULL_CALIBRATE_CMD):
               win.FindElement('-fullCalibrationStatus-').Update("Calibration OK!")   
            else:
               win.FindElement('-fullCalibrationStatus-').Update("Error try again!")       
             
        elif event == "button-lss-set":
            LLS_frame_19(5)     
            
        #br = draw_figure(window["-CANVAS-"].TKCanvas, fig)
      #  a.show()
        br.draw()
       
       # br.draw()
        #plt.pause(5)
       
        if event == "OK" or event == sg.WIN_CLOSED:
            
          break
    stop_threads = True
    updateTaskTerminat = True
    try:
        serTerminal.close()  
    except:
        dummy = "error"
    win.close()        
    

       
       
            
def  exit_gracefully():
    sys.exit()
    raise(SystemExit) 
        
if __name__ == '__main__':
    main()
 
         
             