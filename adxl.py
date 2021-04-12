#!/usr/bin/env python
# coding: utf-8

# 
# [0x24] THRESH_ACT 18 - 0x12
# [0x27] ACT_INACT_CTL    0111 0000 0x70
# [0x2F] INT_MAP 1110 1111   0xEF
# [0x2E] INT_ENABLE 0001 0000  0x10
# 

# In[ ]:


from machine import Pin, SoftI2C
import math
import time
import machine

device = const(0x53)


INT1 = machine.Pin(4, machine.Pin.IN)
INT2 = machine.Pin(2, machine.Pin.IN)
CS = machine.Pin(27, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)
CS.value(1)
LED.value(1)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

time.sleep_ms(500)
i2c.writeto_mem(device, 0x24, b'0x12')
time.sleep_ms(500)
i2c.writeto_mem(device, 0x27, b'0x70')
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2F, b'0xEF')
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2E, b'0x10')
time.sleep_ms(500)



print(INT1.value())


# In[ ]:


from machine import Pin, SoftI2C
import math
import time
import machine

device = const(0x53)


INT1 = machine.Pin(4, machine.Pin.IN)
INT2 = machine.Pin(2, machine.Pin.IN)
CS = machine.Pin(27, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)
CS.value(1)
LED.value(1)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

time.sleep_ms(500)
i2c.writeto_mem(device, 0x24, const(0x12))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x27, const(0x70))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2F, const(0xEF))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2E, const(0x10))
time.sleep_ms(500)


print(INT1.value())


# In[ ]:


from machine import Pin, SoftI2C
import math
import time
import machine

device = const(0x53)


INT1 = machine.Pin(4, machine.Pin.IN)
INT2 = machine.Pin(2, machine.Pin.IN)
CS = machine.Pin(27, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)
CS.value(1)
LED.value(1)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

time.sleep_ms(500)
i2c.writeto_mem(device, 0x24, const(0x12))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x27, const(0x70))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2F, const(0xEF))
time.sleep_ms(500)
i2c.writeto_mem(device, 0x2E, const(0x10))
time.sleep_ms(500)


print(INT1.value())


# In[ ]:





# In[ ]:





# In[ ]:


from machine import Pin
import math
import time
import machine
device = const(0x53)
regAddress = const(0x32)
TO_READ = 6
buff = bytearray(6)



INT1 = machine.Pin(4, machine.Pin.IN)
INT2 = machine.Pin(2, machine.Pin.IN)
CS = machine.Pin(27, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)
CS.value(1)
LED.value(1)



from machine import Pin, SoftI2C

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)





i2c.writeto_mem(device, 0x31, b'0x20')
i2c.writeto_mem(device, 0x1D, b'0x10')
i2c.writeto_mem(device, 0x21, b'0x10')
i2c.writeto_mem(device, 0x2A, b'0x0F')
i2c.writeto_mem(device, 0x2F, b'0x00')
i2c.writeto_mem(device, 0x2E, b'0x40')






    

print(INT1.value())
print(INT2.value())    
    
int1_old = INT1.value()    
while True:
    print("INT1="+str(INT1.value()))


# In[ ]:


from machine import Pin
import math
import time
import machine
device = const(0x53)
regAddress = const(0x32)
TO_READ = 6
buff = bytearray(6)



INT1 = machine.Pin(4, machine.Pin.IN)
INT2 = machine.Pin(2, machine.Pin.IN)
CS = machine.Pin(27, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)
CS.value(1)
LED.value(1)



from machine import Pin, SoftI2C

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)






i2c.writeto_mem(device, 0x31, b'0x01')
i2c.writeto_mem(device, 0x2E, b'0x00')
i2c.writeto_mem(device, 0x25, b'0x0F')
i2c.writeto_mem(device, 0x26, b'0x00')
i2c.writeto_mem(device, 0x27, b'0x07')
i2c.writeto_mem(device, 0x2F, b'0x00')

i2c.writeto_mem(device, 0x2E, b'0x08')




print(INT1.value())
print(INT2.value())    
    
int1_old = INT1.value()    
while True:
    

    if INT1.value()!=int1_old:
        print("INT1="+str(INT1.value()))
        int1_old = INT1.value()

    



# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


(/Put, the, ADXL345, into, +/-, 4G, range, by, writing, the, value, 0x01, to, the, DATA_FORMAT, register.)
writeRegister(DATA_FORMAT, 0x01);


# In[ ]:


(/, Suggested, to, properly, program, accelerometer)
writeRegister(INT_ENABLE, 0x00)


# In[ ]:


(/, Set, a, threshold, of, 3g)
  writeRegister(THRESH_INACT, 0x0F);
  (/, Raise, INACT, event, immeidately, (otherwise, it, would, be, in, 1, second...))
  writeRegister(TIME_INACT,0);
  (/, Consider, all, axes, AND, compare, THRESH_INACT, from, 0, acceleration, (otherwise, it, would, be, differential))
  writeRegister(ACT_INACT_CTL, 0x07);
  
  (/, Send, all, interrrupts, to, INT, 1, (PIN, 2))
  writeRegister(INT_MAP,0);
  (/, Enable, interrupts, for, INACTIVITY, only.)
  writeRegister(INT_ENABLE, 0x08);


# In[ ]:


(/Put, the, ADXL345, into, Measurement, Mode, by, writing, 0x08, to, the, POWER_CTL, register.)
writeRegister(POWER_CTL, 0x08);  //Measurement mode
readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.
}


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


(/Add, the, SPI, library, so, we, can, communicate, with, the, ADXL345, sensor)
#include <SPI.h>

(/Assign, the, Chip, Select, signal, to, pin, 10.)
int CS=10;

(/This, is, a, list, of, some, of, the, registers, available, on, the, ADXL345.)
(/To, learn, more, about, these, and, the, rest, of, the, registers, on, the, ADXL345,, read, the, datasheet!)
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

char THRESH_ACT               = 0x24; // Activity threshold
char THRESH_INACT             = 0x38; // Inactivity threshold to 3g
char TIME_INACT               = 0x26; // time before raising interrupt

char INT_ENABLE               = 0x2E; // Enabling the interrupt lines

char INT_MAP                  = 0x2F;
char ACT_INACT_CTL            = 0x27; // mask byte for controlling

char INT_SOURCE               = 0x30;

(/This, buffer, will, hold, values, read, from, the, ADXL345, registers.)
char values[10];
(/These, variables, will, be, used, to, hold, the, x,y, and, z, axis, accelerometer, values.)
int x,y,z;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  // Create an interrupt that will trigger when inactivity is detected
  attachInterrupt(0, interruptHandler, RISING);

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode

  // Send the inactivity && activity  to PIN 1
  // 0xF7 && 0xEF
  writeRegister(INT_MAP,0xF7 && 0xEF);

  // Set the inactivity threshold to 3g (0x38)
//  writeRegister(THRESH_INACT,0x38);
  writeRegister(THRESH_INACT,1);

  // Raise the inact interrupt immediately after going below threshold
  writeRegister(TIME_INACT,0);
  // Map INACT event (only) to PIN 1
  writeRegister(ACT_INACT_CTL, 0x0F);

  // Enab  le inactivity to generate interrupts
  writeRegister(INT_ENABLE, 0x08);

  readRegister(INT_SOURCE, 1, values); // Clear the INT_SOURCE register

  Serial.println("Waiting for interrupt!");
}

void interruptHandler(){
  // readRegister(INT_SOURCE, 1, values); // Clear the INT_SOURCE register
  Serial.println("something raise an interrupt!");
}

void loop(){
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];

  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);      
  delay(500); 
}

(/This, function, will, write, a, value, to, a, register, on, the, ADXL345.)
(/Parameters:)
(/, char, registerAddress, -, The, register, to, write, a, value, to)
(/, char, value, -, The, value, to, be, written, to, the, specified, register.)
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

(/This, function, will, read, a, certain, number, of, registers, starting, from, a, specified, address, and, store, their, values, in, a, buffer.)
(/Parameters:)
(/, char, registerAddress, -, The, register, addresse, to, start, the, read, sequence, from.)
(/, int, numBytes, -, The, number, of, registers, that, should, be, read.)
(/, char, *, values, -, A, pointer, to, a, buffer, where, the, results, of, the, operation, should, be, stored.)
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  (/If, we're, doing, a, multi-byte, read,, bit, 6, needs, to, be, set, as, well.)
  if(numBytes > 1)address = address | 0x40;

  (/Set, the, Chip, select, pin, low, to, start, an, SPI, packet.)
  digitalWrite(CS, LOW);
  (/Transfer, the, starting, register, address, that, needs, to, be, read.)
  SPI.transfer(address);
  (/Continue, to, read, registers, until, we've, read, the, number, specified,, storing, the, results, to, the, input, buffer.)
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  (/Set, the, Chips, Select, pin, high, to, end, the, SPI, packet.)
  digitalWrite(CS, HIGH);
}


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


_ADXL345_DEFAULT_ADDRESS = const(0x53)  # Assumes ALT address pin low

# Conversion factors
_ADXL345_MG2G_MULTIPLIER = 0.004  # 4mg per lsb
_STANDARD_GRAVITY = 9.80665  # earth standard gravity

_REG_DEVID = const(0x00)  # Device ID
_REG_THRESH_TAP = const(0x1D)  # Tap threshold
_REG_OFSX = const(0x1E)  # X-axis offset
_REG_OFSY = const(0x1F)  # Y-axis offset
_REG_OFSZ = const(0x20)  # Z-axis offset
_REG_DUR = const(0x21)  # Tap duration
_REG_LATENT = const(0x22)  # Tap latency
_REG_WINDOW = const(0x23)  # Tap window
_REG_THRESH_ACT = const(0x24)  # Activity threshold
_REG_THRESH_INACT = const(0x25)  # Inactivity threshold
_REG_TIME_INACT = const(0x26)  # Inactivity time
_REG_ACT_INACT_CTL = const(0x27)  # Axis enable control for [in]activity detection
_REG_THRESH_FF = const(0x28)  # Free-fall threshold
_REG_TIME_FF = const(0x29)  # Free-fall time
_REG_TAP_AXES = const(0x2A)  # Axis control for single/double tap
_REG_ACT_TAP_STATUS = const(0x2B)  # Source for single/double tap
_REG_BW_RATE = const(0x2C)  # Data rate and power mode control
_REG_POWER_CTL = const(0x2D)  # Power-saving features control
_REG_INT_ENABLE = const(0x2E)  # Interrupt enable control
_REG_INT_MAP = const(0x2F)  # Interrupt mapping control
_REG_INT_SOURCE = const(0x30)  # Source of interrupts
_REG_DATA_FORMAT = const(0x31)  # Data format control
_REG_DATAX0 = const(0x32)  # X-axis data 0
_REG_DATAX1 = const(0x33)  # X-axis data 1
_REG_DATAY0 = const(0x34)  # Y-axis data 0
_REG_DATAY1 = const(0x35)  # Y-axis data 1
_REG_DATAZ0 = const(0x36)  # Z-axis data 0
_REG_DATAZ1 = const(0x37)  # Z-axis data 1
_REG_FIFO_CTL = const(0x38)  # FIFO control
_REG_FIFO_STATUS = const(0x39)  # FIFO status
_INT_SINGLE_TAP = const(0b01000000)  # SINGLE_TAP bit
_INT_DOUBLE_TAP = const(0b00100000)  # DOUBLE_TAP bit
_INT_ACT = const(0b00010000)  # ACT bit
_INT_INACT = const(0b00001000)  # INACT bit
_INT_FREE_FALL = const(0b00000100)  # FREE_FALL  bit


class DataRate:  # pylint: disable=too-few-public-methods
    """An enum-like class representing the possible data rates.
    Possible values are
    - ``DataRate.RATE_3200_HZ``
    - ``DataRate.RATE_1600_HZ``
    - ``DataRate.RATE_800_HZ``
    - ``DataRate.RATE_400_HZ``
    - ``DataRate.RATE_200_HZ``
    - ``DataRate.RATE_100_HZ``
    - ``DataRate.RATE_50_HZ``
    - ``DataRate.RATE_25_HZ``
    - ``DataRate.RATE_12_5_HZ``
    - ``DataRate.RATE_6_25HZ``
    - ``DataRate.RATE_3_13_HZ``
    - ``DataRate.RATE_1_56_HZ``
    - ``DataRate.RATE_0_78_HZ``
    - ``DataRate.RATE_0_39_HZ``
    - ``DataRate.RATE_0_20_HZ``
    - ``DataRate.RATE_0_10_HZ``
    """

    RATE_3200_HZ = const(0b1111)  # 1600Hz Bandwidth   140mA IDD
    RATE_1600_HZ = const(0b1110)  #  800Hz Bandwidth    90mA IDD
    RATE_800_HZ = const(0b1101)  #  400Hz Bandwidth   140mA IDD
    RATE_400_HZ = const(0b1100)  #  200Hz Bandwidth   140mA IDD
    RATE_200_HZ = const(0b1011)  #  100Hz Bandwidth   140mA IDD
    RATE_100_HZ = const(0b1010)  #   50Hz Bandwidth   140mA IDD
    RATE_50_HZ = const(0b1001)  #   25Hz Bandwidth    90mA IDD
    RATE_25_HZ = const(0b1000)  # 12.5Hz Bandwidth    60mA IDD
    RATE_12_5_HZ = const(0b0111)  # 6.25Hz Bandwidth    50mA IDD
    RATE_6_25HZ = const(0b0110)  # 3.13Hz Bandwidth    45mA IDD
    RATE_3_13_HZ = const(0b0101)  # 1.56Hz Bandwidth    40mA IDD
    RATE_1_56_HZ = const(0b0100)  # 0.78Hz Bandwidth    34mA IDD
    RATE_0_78_HZ = const(0b0011)  # 0.39Hz Bandwidth    23mA IDD
    RATE_0_39_HZ = const(0b0010)  # 0.20Hz Bandwidth    23mA IDD
    RATE_0_20_HZ = const(0b0001)  # 0.10Hz Bandwidth    23mA IDD
    RATE_0_10_HZ = const(0b0000)  # 0.05Hz Bandwidth    23mA IDD (default value)


class Range:  # pylint: disable=too-few-public-methods
    """An enum-like class representing the possible measurement ranges in +/- G.
    Possible values are
    - ``Range.RANGE_16_G``
    - ``Range.RANGE_8_G``
    - ``Range.RANGE_4_G``
    - ``Range.RANGE_2_G``
    """

    RANGE_16_G = const(0b11)  # +/- 16g
    RANGE_8_G = const(0b10)  # +/- 8g
    RANGE_4_G = const(0b01)  # +/- 4g
    RANGE_2_G = const(0b00)  # +/- 2g (default value)


# In[ ]:


def enable_motion_detection(self, *, threshold=18):
        """
        The activity detection parameters.
        :param int threshold: The value that acceleration on any axis must exceed to\
        register as active. The scale factor is 62.5 mg/LSB.
        If you wish to set them yourself rather than using the defaults,
        you must use keyword arguments::
            accelerometer.enable_motion_detection(threshold=20)
        """
        active_interrupts = self._read_register_unpacked(_REG_INT_ENABLE)

        self._write_register_byte(_REG_INT_ENABLE, 0x0)  # disable interrupts for setup
        self._write_register_byte(
            _REG_ACT_INACT_CTL, 0b01110000
        )  # enable activity on X,Y,Z
        self._write_register_byte(_REG_THRESH_ACT, threshold)
        self._write_register_byte(_REG_INT_ENABLE, _INT_ACT)  # Inactive interrupt only

        active_interrupts |= _INT_ACT
        self._write_register_byte(_REG_INT_ENABLE, active_interrupts)
        self._enabled_interrupts["motion"] = True

    def disable_motion_detection(self):
        """
        Disable motion detection
        """
        active_interrupts = self._read_register_unpacked(_REG_INT_ENABLE)
        active_interrupts &= ~_INT_ACT
        self._write_register_byte(_REG_INT_ENABLE, active_interrupts)
        self._enabled_interrupts.pop("motion")


# In[ ]:


accelerometer.enable_motion_detection(threshold=18)
This line enables the libraries motion detection event. It takes in one variable, the threshold.

The threshold variable is the value that acceleration on all axes must exceed for a motion to be detected. If you find this is too sensitive or not sensitive enough you can modify this value. The scale factor is 62.5 mg, so our example is 18*62.5=1125 mg.


# In[ ]:





# In[ ]:


[0x24] THRESH_ACT
[0x25] THRESH_INACT
[0x26] TIME_INACT
[0x27] ACT_INACT_CTL
[0x2F] INT_MAP
[0x2E] INT_ENABLE

