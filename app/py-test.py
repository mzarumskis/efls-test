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
from queue import Queue 



version = "V-0.0.1"


DELAY_AFTER_RECEIVE = 0.05
LOOP_IN_MS = 10
NO_RECEIVE_TIMOUT_IN_S = 2 * LOOP_IN_MS

isSwitchedToFota = 0
noPacketTimeout = 0
frameErrorCounter = 0 

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

# switch to FOTA mode
def frame_02():
    buff = bytearray(b'\x31\xFE')
    buff.append(9)
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
    getAnyFromRS485()
    time.sleep(DELAY_AFTER_RECEIVE) #delay 
    frame_09()
    data = getDataFromSerial()
    print("Data->RX {0}".format(data.hex().upper()))
    if isPackedValid(data):
         print("Build {0}".format(data[4:data[3]+3]))
       
    else:
        print("Packet ERROR")
    

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
    
    stop_threads = False
    ports = serial.tools.list_ports.comports()
    
    if  platform.system() == "Windows":
        print([port.device for port in ports])
    
    serTerminal = open_serial("COM2")     
    
    t1 = threading.Thread(target = readSerial, args=[serTerminal, serialQ])
        
    t1.start()
    
   
    while True:
        try:
           
           getBuild() 
           
        except KeyboardInterrupt:
           stop_threads = True
           break       
       
            
def  exit_gracefully():
    sys.exit()
    raise(SystemExit) 
        
if __name__ == '__main__':
    main()
 
         
             