#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt
"""

import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import time


PORT = '/dev/ttyUSB0'
master = modbus_rtu.RtuMaster(
            serial.Serial(port=PORT, baudrate=115200, bytesize=8, parity='N', stopbits=1, xonxoff=0)
        )
#PORT = '/dev/ttyp5'

def Control_Mode(Driver_ID, Mode):
    # Mode - 0x01 = 위치제어(상태위치) / 0x02 = 위치제어(절대위치) / 0x03 = 속도제어 / 0x04 = 토크제어
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x200D,output_value = Mode)

def Control_Word(Driver_ID, Word):
    # Word - 0x05 = Quick Stop / 0x06 = Clear faults / 0x07 = Disable / 0x08 = Enable / 0x10 = Start(sync) / 0x11 = Start(Left) / 0x12 = Start(Right)
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x200E,output_value = Word)
    
def Synchronous_control_status(Driver_ID, Status):
    # Status - 0x00 = Synchronous control / 0x01 = Asynchronous control
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x200F,output_value = Status)
    
def Position_Mode_Position_Set(Driver_ID, Position):
    # Left 모터 기준 / 1회전 = 엔코더 분해능 * 4
    position_L = Position & 0xFFFF
    position_H = (Position >> 16) & 0xFFFF
    Data = [position_H, position_L]
    master.execute(Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x208A, output_value=Data)
    


def Position_Mode_Speed_Set(Driver_ID, Speed):
    # 위치제어시 모터 속도 지령 값 Left 모터 기준 / 단위 RPM
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x208E,output_value=Speed)
    
def Speed_Mode_Speed_Set_Left(Driver_ID, Speed):
    # 속도제어시 모터 속도 지령값 Left 모터 기준 / 단위 RPM
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x2088,output_value=Speed)

def Speed_Mode_Speed_Set_Right(Driver_ID, Speed):
    # 속도제어시 모터 속도 지령값 Right 모터 기준 / 단위 RPM
    master.execute(Driver_ID,cst.WRITE_SINGLE_REGISTER,0x2089,output_value=Speed)

def Speed_Mode_Speed_Set_Sync(Driver_ID, Speed_L, Speed_R):
    # 속도제어시 모터 속도 지령값 Left, Right 모터 순서 / 단위 RPM
    master.execute(Driver_ID,cst.WRITE_MULTIPLE_REGISTERS,0x2088,output_value=[Speed_L,Speed_R])
    
def Read_actual_velocity_left(Driver_ID):
    # 현재 모터 속도값 Left 모터 / 단위 0.1RPM
    value = master.execute(Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20AB, 1)
    return convert_i16(value[0])/10

def Read_actual_position_left(Driver_ID):
    # 현재 모터 위치값 Left 모터 / 1회전 = 엔코더 분해능 * 4
    valueh = master.execute(Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20A7, 1)
    valuel = master.execute(Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20A8, 1)
    print(valueh, valuel)
    return convert_to_signed_32bit(valueh[0], valuel[0])

def convert_i16(value):
    return value - 65536 if value > 32767 else value

def convert_to_signed_32bit(high, low):
    value = (high << 16) | low  # 상위 비트를 왼쪽으로 16bit shift 후, 하위 비트와 결합
    if value >= (1 << 31):  # 부호 있는 32비트 변환
        value -= (1 << 32)
    return value

def main():
    """main"""
    logger = modbus_tk.utils.create_logger("console")

    try:
        #Connect to the slave
        
        master.set_timeout(5.0)
        master.set_verbose(True)
        logger.info("connected")

        logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 0x2000, 3))
        
        
        Control_Mode(1, 0x03)
        Control_Word(1, 0x08)
        
        Speed_Mode_Speed_Set_Sync(1, 20, -20)
        time.sleep(3)
        vel = Read_actual_velocity_left(1)
        time.sleep(0.5)
        logger.info(vel)
        
        Speed_Mode_Speed_Set_Sync(1, 50, -50)
        time.sleep(3)
        vel = Read_actual_velocity_left(1)
        time.sleep(0.5)
        logger.info(vel)
        
        
        
        # Speed_Mode_Speed_Set_Left(1, 100)    # 350까지는 안정
        # Speed_Mode_Speed_Set_Right(1, -30)    # 350까지는 안정
        
        # time.sleep(2)

        
        # # position mode
        # Control_Mode(1,0x02)
        # Control_Word(1,0x08)
        # Position_Mode_Position_Set(1,-1000)
        # Position_Mode_Speed_Set(1,200)
        # Control_Word(1,0x10)
        
        # time.sleep(3)
        # logger.info(Read_actual_position_left(1))
        
        Control_Word(1, 0x07)
        

    except modbus_tk.modbus.ModbusError as exc:
        logger.error("%s- Code=%d", exc, exc.get_exception_code())

if __name__ == "__main__":
    main()

