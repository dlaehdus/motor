#!/usr/bin/env python
# -*- coding: utf_8 -*-

import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

class ZlacController:
    def __init__(self, port_name=PORT, baudrate=BAUDRATE, driver_id=1):
        self.Driver_ID = driver_id
        
        self.logger = modbus_tk.utils.create_logger("console")
        
        #Connect to the slave
        self.master = modbus_rtu.RtuMaster(
            serial.Serial(port=port_name, baudrate=baudrate, bytesize=8, parity='N', stopbits=1, xonxoff=0)
        )
        self.master.set_timeout(5.0)
        self.master.set_verbose(True)
        self.logger.info("connected")

        self.logger.info(self.master.execute(1, cst.READ_HOLDING_REGISTERS, 0x2000, 3))
    
    def Control_word(self, word):
        # word - 0x05 = Quick Stop / 0x06 = Clear faults / 0x07 = Stop / 0x08 = Enable / 0x10 = Start(sync) / 0x11 = Start(Left) / 0x12 = Start(Right)
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x200E, output_value=word)
    
    def Control_mode(self, mode):
        # mode - 0x01 = 위치제어(상대위치) / 0x02 = 위치제어(절대위치) / 0x03 = 속도제어 / 0x04 = 토크제어
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x200D, output_value=mode)
    
    def Synchronous_control_status(self, status):
        # status - 0x00 = 동기 / 0x01 = 비동기
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x200F, output_value=status)
    
    def Target_velocity_left(self, velocity):
        # 속도제어시 모터 속도 지령값 Left 모터 기준 / 단위 RPM / range: -3000 ~ 3000
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x2088, output_value=velocity)
    
    def Target_velocity_right(self, velocity):
        # 속도제어시 모터 속도 지령값 Right 모터 기준 / 단위 RPM / range: -3000 ~ 3000
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x2089, output_value=velocity)
    
    def Target_velocity_sync(self, velocity_L, velocity_R):
        # 속도제어시 모터 속도 지령값 Left, Right 모터 순서 / 단위 RPM / range: -3000 ~ 3000
        self.master.execute(self.Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x2088, output_value=[velocity_L, velocity_R])
    
    def Read_actual_velocity_left(self):
        # 현재 모터 속도값 Left 모터 / 단위 0.1RPM
        value = self.master.execute(self.Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20AB, 1)
        return self.convert_i16(value)

    def Read_actual_velocity_right(self):
        # 현재 모터 속도값 Right 모터 / 단위 0.1RPM
        value = self.master.execute(self.Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20AC, 1)
        return self.convert_i16(value)
    
    def Target_position_left(self, Position):
        # Left 모터 기준 / 1회전 = 엔코더 분해능 * 4
        position_L = Position & 0xFFFF
        position_H = (Position >> 16) & 0xFFFF
        Data = [position_H, position_L]
        self.master.execute(self.Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x208A, output_value=Data)
    
    def Target_position_right(self, Position):
        # Right 모터 기준 / 1회전 = 엔코더 분해능 * 4
        position_L = Position & 0xFFFF
        position_H = (Position >> 16) & 0xFFFF
        Data = [position_H, position_L]
        self.master.execute(self.Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x208C, output_value=Data)
    
    def Target_position_sync(self, Position_L, Position_R):
        # Left, Right 모터 순서 / 1회전 = 엔코더 분해능 * 4
        position_L_L = Position_L & 0xFFFF
        position_L_H = (Position_L >> 16) & 0xFFFF
        position_R_L = Position_R & 0xFFFF
        position_R_H = (Position_R >> 16) & 0xFFFF
        Data = [position_L_H, position_L_L, position_R_H, position_R_L]
        self.master.execute(self.Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x208A, output_value=Data)
    
    def Target_position_speed_left(self, speed):
        # 위치제어시 모터 속도 지령값 Left 모터 기준 / 단위 RPM / range: 0 ~ 1000
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x208E, output_value=speed)
    
    def Target_position_speed_right(self, speed):
        # 위치제어시 모터 속도 지령값 Right 모터 기준 / 단위 RPM / range: 0 ~ 1000
        self.master.execute(self.Driver_ID, cst.WRITE_SINGLE_REGISTER, 0x208F, output_value=speed)
    
    def Target_position_speed_sync(self, speed_L, speed_R):
        # 위치제어시 모터 속도 지령값 Left, Right 모터 순서 / 단위 RPM / range: 0 ~ 1000
        self.master.execute(self.Driver_ID, cst.WRITE_MULTIPLE_REGISTERS, 0x208E, output_value=[speed_L, speed_R])
        
    def Read_actual_position_left(self):
        # 현재 모터 위치값 Left 모터
        value = self.master.execute(self.Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20A7, 2)
        return self.convert_to_signed_32bit(value[0], value[1])
    
    def Read_actual_position_right(self):
        # 현재 모터 위치값 Right 모터
        value = self.master.execute(self.Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20A9, 2)
        return self.convert_to_signed_32bit(value[0], value[1])
    
    def Read_actual_position_sync(self):
        # 현재 모터 위치값 Left, Right 모터 순서
        value = self.master.execute(self.Driver_ID, cst.READ_HOLDING_REGISTERS, 0x20A7, 4)
        return self.convert_to_signed_32bit(value[0], value[1]), self.convert_to_signed_32bit(value[2], value[3])
    
    def convert_to_signed_32bit(self, high, low):
        value = (high << 16) | low  # 상위 비트를 왼쪽으로 16bit shift 후, 하위 비트와 결합
        if value >= (1 << 31):  # 부호 있는 32비트 변환
            value -= (1 << 32)
        return value
    
    def convert_i16(self, value):
        return value - 65536 if value > 32767 else value
        