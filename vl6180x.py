# 
# MIT License
# AUTHORS : https://github.com/Ledbelly2142
# https://github.com/Ledbelly2142/VL6180X/blob/master/vl6180x.py

#
#Copyright (c) 2017 Greg

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

# Modification par Mace Robotics


import ustruct
import struct
import time
from machine import I2C

# i2c = I2C(0, I2C.MASTER, baudrate=100000, pins=('P8', 'P9'))

class Sensor:
    
    __ALS_GAIN_1    = 0x06
    __ALS_GAIN_1_25 = 0x05
    __ALS_GAIN_1_67 = 0x04
    __ALS_GAIN_2_5  = 0x03
    __ALS_GAIN_5    = 0x02
    __ALS_GAIN_10   = 0x01
    __ALS_GAIN_20   = 0x00
    __ALS_GAIN_40   = 0x07
    
    __VL6180X_SYSTEM_INTERRUPT_CLEAR = 0x015
    __VL6180X_SYSALS_START = 0x038
    __VL6180X_RESULT_ALS_VAL = 0x050
    __VL6180X_SYSALS_ANALOGUE_GAIN = 0x03F
    
    # Dictionaries with the valid ALS gain values
    # These simplify and clean the code (avoid abuse of if/elif/else clauses)
    ALS_GAIN_REG = {
        1:      __ALS_GAIN_1,
        1.25:   __ALS_GAIN_1_25,
        1.67:   __ALS_GAIN_1_67,
        2.5:    __ALS_GAIN_2_5,
        5:      __ALS_GAIN_5,
        10:     __ALS_GAIN_10,
        20:     __ALS_GAIN_20,
        40:     __ALS_GAIN_40
    }
    
    ALS_GAIN_ACTUAL = {    # Data sheet shows gain values as binary list
        1:      1.01,      # Nominal gain 1;    actual gain 1.01
        1.25:   1.28,      # Nominal gain 1.25; actual gain 1.28
        1.67:   1.72,      # Nominal gain 1.67; actual gain 1.72
        2.5:    2.60,      # Nominal gain 2.5;  actual gain 2.60
        5:      5.21,      # Nominal gain 5;    actual gain 5.21
        10:     10.32,     # Nominal gain 10;   actual gain 10.32
        20:     20.00,     # Nominal gain 20;   actual gain 20
        40:     40.00,     # Nominal gain 40;   actual gain 40
    }
    
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self._address = address
        self.default_settings()
        self.init()

    def myWrite16(self, register, regValue):
        """ write a byte to specified 16 bit register """
        return self.i2c.writeto_mem(self._address, register, bytearray([regValue]), addrsize=16), 'big'
    
    def myWrite8(self, register_address, data):
        return self.i2c.writeto_mem(self._address, register_address, bytearray([data]), addrsize=8), 'big'

    

    def myRead16(self, register):
        """read 1 bit from 16 byte register"""
        # i2c.readfrom_mem(0x29, 0x0016, 1, addrsize=16)
        value = int.from_bytes(
            self.i2c.readfrom_mem(self._address, register, 1, addrsize=16),
            'big'
        )
        return value & 0xFFFF

    def init(self):
        if self.myRead16(0x0016) != 1:
            raise RuntimeError("Failure reset")

        # Recommended setup from the datasheet
     
        self.myWrite16(0x0207, 0x01)
        self.myWrite16(0x0208, 0x01)
        self.myWrite16(0x0096, 0x00)
        self.myWrite16(0x0097, 0xfd)
        self.myWrite16(0x00e3, 0x00)
        self.myWrite16(0x00e4, 0x04)
        self.myWrite16(0x00e5, 0x02)
        self.myWrite16(0x00e6, 0x01)
        self.myWrite16(0x00e7, 0x03)
        self.myWrite16(0x00f5, 0x02)
        self.myWrite16(0x00d9, 0x05)
        self.myWrite16(0x00db, 0xce)
        self.myWrite16(0x00dc, 0x03)
        self.myWrite16(0x00dd, 0xf8)
        self.myWrite16(0x009f, 0x00)
        self.myWrite16(0x00a3, 0x3c)
        self.myWrite16(0x00b7, 0x00)
        self.myWrite16(0x00bb, 0x3c)
        self.myWrite16(0x00b2, 0x09)
        self.myWrite16(0x00ca, 0x09)
        self.myWrite16(0x0198, 0x01)
        self.myWrite16(0x01b0, 0x17)
        self.myWrite16(0x01ad, 0x00)
        self.myWrite16(0x00ff, 0x05)
        self.myWrite16(0x0100, 0x05)
        self.myWrite16(0x0199, 0x05)
        self.myWrite16(0x01a6, 0x1b)
        self.myWrite16(0x01ac, 0x3e)
        self.myWrite16(0x01a7, 0x1f)
        self.myWrite16(0x0030, 0x00)
        
        self.myWrite16(0x0011, 0x10) # Enables polling for ‘New Sample ready’ when measurement completes
        self.myWrite16(0x010a, 0x30) # Set the averaging sample period (compromise between lower noise and increased execution time)
        self.myWrite16(0x003f, 0x46) # Sets the light and dark gain (upper nibble). Dark gain should not be changed.
        self.myWrite16(0x0031, 0xFF) # sets the # of range measurements after which auto calibration of system is performed
        self.myWrite16(0x0040, 0x63) # Set ALS integration time to 100ms
        self.myWrite16(0x002e, 0x01) # perform a single temperature calibratio of the ranging sensor
        self.myWrite16(0x001b, 0x09) # Set default ranging inter-measurement period to 100ms
        self.myWrite16(0x003e, 0x31) # Set default ALS inter-measurement period to 500ms
        self.myWrite16(0x0014, 0x24) # Configures interrupt on ‘New Sample Ready threshold event’ 
        

#  writeReg System__Fresh_OUT_OF_Reset
        # self.myWrite16(0x0016, 0x00),

    def default_settings(self):
        # Enables polling for ‘New Sample ready’ when measurement completes
        self.myWrite16(0x0011, 0x10)
        self.myWrite16(0x010A, 0x30)  # Set Avg sample period
        self.myWrite16(0x003f, 0x46)  # Set the ALS gain
        self.myWrite16(0x0031, 0xFF)  # Set auto calibration period
        # (Max = 255)/(OFF = 0)
        self.myWrite16(0x0040, 0x63)  # Set ALS integration time to 100ms
        # perform a single temperature calibration
        self.myWrite16(0x002E, 0x01)

        # Optional settings from datasheet
        self.myWrite16(0x001B, 0x09)  # Set default ranging inter-measurement
        # period to 100ms
        self.myWrite16(0x003E, 0x0A)  # Set default ALS inter-measurement
        # period to 100ms
        self.myWrite16(0x0014, 0x24)  # Configures interrupt on ‘New Sample
        # Ready threshold event’

        # Additional settings defaults from community
        self.myWrite16(0x001C, 0x32)  # Max convergence time
        self.myWrite16(0x002D, 0x10 | 0x01)  # Range check enables
        self.myWrite16(0x0022, 0x7B)  # Eraly coinvergence estimate
        self.myWrite16(0x0120, 0x01)  # Firmware result scaler

    def identify(self):
        """Retrieve identification information of the sensor."""
        return {
            'model': self.myRead16(0x0000),
            'revision': (self.myRead16(0x0001), self.myRead16(0x0002)),
            'module_revision': (self.myRead16(0x0003),
                                self.myRead16(0x0004)),
            'date': self.myRead16(0x006),
            'time': self.myRead16(0x008),
        }

    def address(self, address=None):
        """Change the I2C address of the sensor."""
        if address is None:
            return self._address
        if not 8 <= address <= 127:
            raise ValueError("Wrong address")
        self._set_reg8(0x0212, address)
        self._address = address

    def range(self):
        """Measure the distance in millimeters. Takes 0.01s."""
        self.myWrite16(0x0018, 0x01)  # Sysrange start
        time.sleep(0.01)
        return self.myRead16(0x0062)  # Result range valueimport ustruct
        #return self.myRead16(0x0074) 
    
    def get_als(self, als_gain):
        print("1111111")
        
        self.myWrite16(0x0018, 0x01)  # Sysrange start
        
        reg = self.myRead16(0x014)
        
        reg &= ~0x38
        
        reg |= 0x4 << 3  # IRQ on ALS ready
        
        print("reg = ", reg)
        
        self.myWrite16(0x014, reg)
        
        reg = self.myRead16(0x014)
        print("reg 0x014 = ", reg)
        
        reg = self.myRead16(0x04E)
        print("reg RESULT__ALS_STATUS = ", reg)
        
        # 100 ms integration period
        self.myWrite16(0x040, 0)
        self.myWrite16(0x041, 100)
        
        reg = self.myRead16(0x041)
        
        print("reg 0x041 = ", reg)
        
        if als_gain not in self.ALS_GAIN_ACTUAL:
            print("Invalid gain setting: %d.  Setting to 20." % als_gain)
        
        reg = self.myRead16(0x04E)
        print("reg RESULT__ALS_STATUS = ", reg)
        
        print("222222222")
        als_gain_actual = self.ALS_GAIN_ACTUAL.setdefault(als_gain, 20)
        print("als_gain_actual = ", als_gain_actual)
        
        print("44444 = ",self.ALS_GAIN_REG.setdefault(als_gain, self.__ALS_GAIN_20))
        
        a = (0x40 | self.ALS_GAIN_REG.setdefault(als_gain, self.__ALS_GAIN_20))
        
        print("a = ", a)
        
        self.myWrite16(0x03F, a)
        
        reg = self.myRead16(0x03F)
        print("reg 0x03F = ", reg)
        
        reg = self.myRead16(0x04D) >> 4
        print("reg _REG_RESULT_RANGE_STATUS  = ", reg)
        
        
        self.myWrite16(0x0018, 0x01)  # Sysrange start
        
        
        # Start ALS Measurement
        self.myWrite16(0x038, 0x1)
        
        start= self.myRead16(0x038)
        
        print("start = ",start )
        
        time.sleep(0.500)   # sleep
        
        als_raw = self.myRead16(0x050)
        
        print("7777777777 als_raw = ", als_raw)
        
        self.myWrite16(0x015, 0x07)
        
        
        
        als_integration_period_raw = self.myRead16(0x0040)
        
        print("als_integration_period_raw = ", als_integration_period_raw)
        
        als_integration_period = 100.0 / als_integration_period_raw
        
        print("999 als_integration_period = ", als_integration_period)
        # Calculate actual LUX from application note
        als_calculated = 0.32 * (als_raw / als_gain_actual) * als_integration_period

        return als_gain_actual
        
        
        
# end of file