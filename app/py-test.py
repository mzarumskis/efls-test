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



version = "V-0.0.1"

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
            data = getDataFromSerialWithTimeout(0.05)
            
            if isPackedValid(data):
               return True 
        except :
            dummy="e"
   
   # data = getDataFromSerial()

def getBuild():
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
                 break
        else:
            print("Packet ERROR")
            

def switchToSlaveMode():
        
    tryCount = 3
    while(tryCount):
        getAnyFromRS485()
        time.sleep(DELAY_AFTER_RECEIVE) #delay 
        frame_02()
        data = getDataFromSerial()
        print("Data->RX {0}".format(data.hex().upper()))
        tryCount = tryCount - 1
        if isPackedValid(data):
             if isSwticToSlavedAckFrame(data):
                 print("Slave Mode Entered")
                 break
        else:
            print("Packet ERROR")
            
            
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
    
    
    
def getBinFile():
    dir_path = os.path.dirname(os.path.realpath(__file__)) 
    for d, subD, f in os.walk(dir_path):
        for file in f:
            if file.endswith('.bin'): 
                return file
    return ""

def getVersionFromFileName(fileName):
    if fileName != "":
        print("File found: {0}".format(fileName))
        name = fileName.split("_")
        version = name[1].split(".")
        data = bytes.fromhex(version[0]+ version[1]+version[2])
        return data
        
    return ""
        
        
def fwUploadTask(updateFotaHeaderOnly):
    print("Opne file")
    fileName = getBinFile()
    if fileName != "":
        
        if not updateFotaHeaderOnly:
            
                file = open(fileName, "rb")
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
                        print("Sent {0}%".format(round((bytesSent/fileSize) * 100)))
                        
                                     
                    else:
                        version = getVersionFromFileName(fileName)    
                        frame_11_writeFotaHeader(version)   
                        waitAckPacket()
                        break
        else:
            version = getVersionFromFileName(fileName)    
            frame_11_writeFotaHeader(version)   
                            

def main():

    global isSwitchedToFota
    global noPacketTimeout
    global frameErrorCounter
    global stop_threads 
    global serialQ
    global serTerminal
    global bootForce
    global boudRate
    
    bootForce = False
    boudRate = RS485_BOUD_RATE_APP
    n = len(sys.argv)
    for i in range(1, n):
        print(sys.argv[i], end = " \r\n")
        if sys.argv[i] == "boot":
            bootForce = True
            boudRate = RS485_BOUD_RATE_BOOT_ONLY
    
    serialQ = Queue() 
    
    print("EFLS Sensor Test environment {0}".format(version))
    print("Platform {0}".format(platform.system()))
    
    developMode = False
    
    if  developMode:
        
        getVersionFromFileName()
        
        time.sleep(1)   
    else:
       
            
        stop_threads = False
        ports = serial.tools.list_ports.comports()
        
        if  platform.system() == "Windows":
            print([port.device for port in ports])
        
        serTerminal = open_serial("COM2", boudRate)     
        
        t1 = threading.Thread(target = readSerial, args=[serTerminal, serialQ])
            
        t1.start()
        if bootForce:
            switchToFwUpdate()
        
        getBuild() 
        switchToSlaveMode()
        fwUploadTask(False)
       
            
    
    while True:
        try:
           
           time.sleep(1)   
           
        except KeyboardInterrupt:
           stop_threads = True
           break       
       
            
def  exit_gracefully():
    sys.exit()
    raise(SystemExit) 
        
if __name__ == '__main__':
    main()
 
         
             