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

FOTA_BLOCK_SZIE_IN_BYTES = 16

DELAY_AFTER_RECEIVE = 0.05
LOOP_IN_MS = 10
NO_RECEIVE_TIMOUT_IN_S = 2 * LOOP_IN_MS

isSwitchedToFota = 0
noPacketTimeout = 0
frameErrorCounter = 0 


#Command definitions

WRITE_FOTA_RECORD = 4

def open_serial(port):
    ser = serial.Serial(port, 19200, timeout=0.1)
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
    buff.append(size+2)
    buff.append((recordIdx >> 8) & 0xFF)
    buff.append(recordIdx & 0xFF)
    
    for b in data:
        buff.append(b)
        
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    #serTerminal.write(buff)    


def getDataFromSerial():
    data = serialQ.get()
    if data:
        return data
    time.sleep(1/LOOP_IN_MS) #delay 50mS    

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
            
def getBinFile():
    dir_path = os.path.dirname(os.path.realpath(__file__)) 
    for d, subD, f in os.walk(dir_path):
        for file in f:
            if file.endswith('.bin'): 
                return file
    return ""
def main():

    global isSwitchedToFota
    global noPacketTimeout
    global frameErrorCounter
    global stop_threads 
    global serialQ
    global serTerminal
    
    serialQ = Queue() 
    
    print("EFLS Sensor Test environment {0}".format(version))
    print("Platform {0}".format(platform.system()))
    
    developMode = True
    
    if  developMode:
        
        print("Opne file")
        fileName = getBinFile()
        if fileName != "":
            print("File found: {0}".format(fileName))  
            file = open(fileName, "rb")
            recordIdx =0
            while True:
                data = file.read(FOTA_BLOCK_SZIE_IN_BYTES)
                if data:
                    #print("File found: {0:X}:{1}".format(recordIdx,data.hex().upper()))   
                    frame_04_writeRecordToFlash(recordIdx, len(data), data)
                    
                    recordIdx = recordIdx +1
                else:
                    break
        
        time.sleep(1)   
    else:
        stop_threads = False
        ports = serial.tools.list_ports.comports()
        
        if  platform.system() == "Windows":
            print([port.device for port in ports])
        
        serTerminal = open_serial("COM2")     
        
        t1 = threading.Thread(target = readSerial, args=[serTerminal, serialQ])
            
        t1.start()
        
        getBuild() 
        switchToSlaveMode()
    
    
    
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
 
         
             