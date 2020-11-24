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

version = "V-0.0.1"

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
def frame_02(terminal):
    buff = bytearray(b'\x31\xFE')
    buff.append(2)
    buff.append(0)
    crc = AddCrc(buff)
    buff.append(crc)
    print("Data->TX {0}".format(buff.hex().upper()))
    terminal.write(buff)    

def main():

    global isSwitchedToFota
    global noPacketTimeout
    global frameErrorCounter
    
    print("EFLS Sensor Test environment {0}".format(version))
    print("Platform {0}".format(platform.system()))
    
    ports = serial.tools.list_ports.comports()
    
    if  platform.system() == "Windows":
        print([port.device for port in ports])
    
    ser = open_serial("COM2")     
        
    while (ser.isOpen()):
        rawdata = ser.read(64)
        
        if rawdata:
            print("Data->RX {0}".format(rawdata.hex().upper()))
            if not isSwitchedToFota:
                time.sleep(0.05) #delay 50mS
                frame_02(ser)
                noPacketTimeout = 0
                #isSwitchedToFota =1
        
        noPacketTimeout = noPacketTimeout + 1    
        time.sleep(1.0/LOOP_IN_MS)
        if noPacketTimeout > NO_RECEIVE_TIMOUT_IN_S:
            noPacketTimeout = 0
            frameErrorCounter = frameErrorCounter + 1
            print("RS485 No Frame ERROR {}".format(frameErrorCounter))
            frame_02(ser)
        
        
if __name__ == '__main__':
   main()