'''
Created on Oct 14, 2020

@author: mzaru
'''
import serial
from serial.tools.list_ports import comports
import time
import sys
import binascii
import crc8
import os


FOTA_BLOCK_SZIE_IN_BYTES = 64
BLOCK_BEFOR_CRC_ANDSIZE_BYTES = 192
BOOT_RECORD_SIZE_BYTES = 16

#DIR_ = "\Debug"
DIR_ = ""

version = "V-0.0.1"


      

def getBinFile():
    dir_path = os.path.dirname(os.path.realpath(__file__)) +DIR_
    print("File path: {0}".format(dir_path))
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


def crc16(data : bytearray, offset , length, crcIni):
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    crc = crcIni
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0,8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc & 0xFFFF
        
        
def convertFile():
    crcOut = 0xFFFF
    print("Opne file")
    fileName = getBinFile()
    if fileName != "":
   
        file = open(fileName, "rb+")
        recordIdx =0
        fileSize = len(file.read())
        print("File found: {0} szie {1}".format(fileName, fileSize))
        file.seek(0)      
  
        data = file.read()
        if data:
            print("Data Len: {0}".format(hex(len(data))))
            
            crcOut = crc16(data, 0, BLOCK_BEFOR_CRC_ANDSIZE_BYTES, crcOut)    
            crcOut = crc16(data, BLOCK_BEFOR_CRC_ANDSIZE_BYTES + BOOT_RECORD_SIZE_BYTES, fileSize - BLOCK_BEFOR_CRC_ANDSIZE_BYTES - BOOT_RECORD_SIZE_BYTES, crcOut)    
            idx = BLOCK_BEFOR_CRC_ANDSIZE_BYTES
            
            outdata = bytearray(data)
        
            outdata [idx] = (fileSize) & 0xFF
            idx = idx + 1
            outdata [idx] = (fileSize >> 8) & 0xFF
            idx = idx + 1
            outdata [idx] = (fileSize >> 16) & 0xFF
            idx = idx + 1
            outdata [idx] = (fileSize >> 24) & 0xFF
            
            idx = idx + 1
            outdata [idx] = (crcOut) & 0xFF
            idx = idx + 1
            outdata [idx] = (crcOut>> 8) & 0xFF
            
            file.seek(0)     
            file.write(outdata)
            
            print("File CRC = {0}".format(hex(crcOut)))
                         
                             
        file.close()   
             
              
                                 

def main():

    global isSwitchedToFota
    global noPacketTimeout
    global frameErrorCounter
    global stop_threads 
    global serialQ
    global serTerminal
        
    print("EFLS Sensor bin converter {0}".format(version))
        
    convertFile()
   
   
       
            
def  exit_gracefully():
    sys.exit()
    raise(SystemExit) 
        
if __name__ == '__main__':
    main()
 
         
             