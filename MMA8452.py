#!/usr/bin/env python
# coding: utf-8

# In[ ]:


https://www.nxp.com/docs/en/application-note/AN4071.pdf


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





# In[ ]:





# In[ ]:


Motion is often used to simply alert the main processor that the device is currently in use. When the acceleration exceeds a
set threshold the motion interrupt is asserted. A motion can be a fast moving shake or a slow moving tilt. This will depend on the
threshold and timing values configured for the event. The motion detection function can analyze static acceleration changes or
faster jolts. For example, to detect that an object is spinning, all three axes would be enabled with a threshold detection of > 2g.
This condition would need to occur for a minimum of 100 ms to ensure that the event wasn't just noise. The timing value is set
by a configurable debounce counter. The debounce counter acts like a filter to determine whether the condition exists for
configurable set of time (i.e., 100 ms or longer). There is also directional data available in the source register to detect the
direction of the motion. This is useful for applications such as directional shake or flick, which assists with the algorithm for various
gesture detections.


# In[ ]:


void MMA8452::initMMA8452(unsigned char fsr, unsigned char dr, unsigned char sr, unsigned char sc, unsigned char mt, unsigned char mdc)
{
    MMA8452Standby();

//Set up the full scale range to 2, 4, or 8g.
if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(XYZ_DATA_CFG, fsr >> 2);  
else
    writeRegister(XYZ_DATA_CFG, 0);

writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0xF8));
if (dr<= 7)
    writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (dr << DR0));
if (sr<=3)
    writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (sr << ASLP_RATE0));

writeRegister(CTRL_REG2, readRegister(CTRL_REG2) | (LP << SMODS) | (LP << MODS) | (1 << SLPE));  //LOW POWER MODE IN SLEEP & ACTIVE STATES WITH LOWEST SAMPLING
writeRegister(ASLP_COUNT, sc);

// Set up interrupt 1 and 2: 1 = wake ups, 2 = data
writeRegister(CTRL_REG3, (1 << WAKE_FF_MT) | (1 << IPOL));  // Active high, push-pull interrupts, sleep wake up from motion detection
writeRegister(CTRL_REG4, (1 << INT_EN_ASLP) | (1 << INT_EN_FF_MT) |  (1 << INT_EN_DRDY));  // DRDY ENABLE SLP/AWAKE INTERRUPT (INTERRUPT THROWN WHENEVER IT CHANGES) & MOTION INTERRUPT TO KEEP AWAKE
writeRegister(CTRL_REG5, (1 << INT_CFG_ASLP) | (1 << INT_CFG_FF_MT));  // DRDY on INT1, ASLP_WAKE INT2, FF INT2
writeRegister(CTRL_REG5, readRegister(CTRL_REG5));

//SETUP THE MOTION DETECTION
writeRegister(FF_MT_CFG, 0xF8);     /*MOTION DETECTION AND LATCH THE 
                                    //RESULT WHEN IT HAPPENS AS OPPOSED 
                                    //TO COMBINATIONAL REAL TIME*/
    writeRegister(FF_MT_THS, mt);       //MOTION DETECTION THRESHOLDS 
    writeRegister(FF_MT_COUNT, mdc);    //TIME MOTION NEEDS TO BE 
                                        //PRESENT ABOVE THE THRESHOLD BEFORE INTERRUPT CAN BE ASSERTED

    MMA8452Active();
}


# In[ ]:


Is the Sensor in Standby Mode?  You can only change the registers if the Device is in StandBy Mode.
(/, Sets, the, MMA8452, to, standby, mode.)
(/, It, must, be, in, standby, to, change, most, register, settings)
void MMA8452Standby()
{
  byte c = readRegister(0x2A);
  writeRegister(CTRL_REG1, c & ~(0x01));
}


# In[ ]:


#define STATUS           0x00
#define OUT_X_MSB        0x01    
#define OUT_X_LSB        0x02
#define OUT_Y_MSB        0x03
#define OUT_Y_LSB        0x04
#define OUT_Z_MSB        0x05
#define OUT_Z_LSB        0x06
#define SYSMOD           0x0B
#define INT_SOURCE       0x0C
#define WHO_AM_I         0x0D   
#define XYZ_DATA_CFG     0x0E
#define HP_FILTER_CUTOFF 0x0F
#define PL_STATUS        0x10
#define PL_CFG           0x11
#define PL_COUNT         0x12
#define PL_BF_ZCOMP      0x13
#define P_L_THS_REG      0x14
#define FF_MT_CFG        0x15   X111 1000   x=0 geçici interrupt x=1 kalıcı interrupt
#define FF_MT_SRC        0x16
#define FF_MT_THS        0x17   0000 0000 son 7 hanesi treshold örn 0000 1111
#define FF_MT_COUNT      0x18
#define TRANSIENT_CFG    0x1D
#define TRANSIENT_SRC    0x1E
#define TRANSIENT_THS    0x1F
#define TRANSIENT_COUNT  0x20
#define PULSE_CFG        0x21
#define PULSE_SRC        0x22
#define PULSE_THSX       0x23
#define PULSE_THSY       0x24
#define PULSE_THSZ       0x25
#define PULSE_TMLT       0x26
#define PULSE_LTCY       0x27
#define PULSE_WIND       0x28
#define ASLP_COUNT       0x29
#define CTRL_REG1        0x2A
#define CTRL_REG2        0x2B
#define CTRL_REG3        0x2C
#define CTRL_REG4        0x2D 0000 0100
#define CTRL_REG5        0x2E 0000 0100
#define OFF_X            0x2F
#define OFF_Y            0x30
#define OFF_Z            0x31


# In[ ]:


(/, Set, up, motion, detection)
  writeRegister(FF_MT_CFG, 0x58); // Set motion flag on x and y axes
  writeRegister(FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
  writeRegister(FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz


# In[ ]:


(/, Set, up, interrupt, 1, and, 2)
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
     
 (/, writeRegister(0x2C,, 0x02);, //, Active, high,, push-pull, interrupts)

  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
   
  writeRegister(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2


# In[ ]:


import smbus
import time
# Get I2C bus
bus = smbus.SMBus(1)
# I2C address of the device
MMA8452Q_DEFAULT_ADDRESS = 0x1C
# MMA8452Q Register Map
MMA8452Q_REG_STATUS = 0x00 # Data status Register
MMA8452Q_REG_OUT_X_MSB = 0x01 # Output Value X MSB
MMA8452Q_REG_OUT_X_LSB = 0x02 # Output Value X LSB
MMA8452Q_REG_OUT_Y_MSB = 0x03 # Output Value Y MSB
MMA8452Q_REG_OUT_Y_LSB = 0x04 # Output Value Y LSB
MMA8452Q_REG_OUT_Z_MSB = 0x05 # Output Value Z MSB
MMA8452Q_REG_OUT_Z_LSB = 0x06 # Output Value Z LSB
MMA8452Q_REG_SYSMOD = 0x0B # System mode Register
MMA8452Q_REG_INT_SOURCE = 0x0C # System Interrupt Status Register
MMA8452Q_REG_WHO_AM_I = 0x0D # Device ID Register
MMA8452Q_REG_XYZ_DATA_CFG = 0x0E # Data Configuration Register
MMA8452Q_REG_CTRL_REG1 = 0x2A # Control Register 1
MMA8452Q_REG_CTRL_REG2 = 0x2B # Control Register 2
MMA8452Q_REG_CTRL_REG3 = 0x2C # Control Register 3
MMA8452Q_REG_CTRL_REG4 = 0x2D # Control Register 4
MMA8452Q_REG_CTRL_REG5 = 0x2E # Control Register 5
# MMA8452Q Data Configuration Register
MMA8452Q_DATA_CFG_HPF_OUT = 0x10 # Output Data High-Pass Filtered
MMA8452Q_DATA_CFG_FS_2 = 0x00 # Full-Scale Range = 2g
MMA8452Q_DATA_CFG_FS_4 = 0x01 # Full-Scale Range = 4g
MMA8452Q_DATA_CFG_FS_8 = 0x02 # Full-Scale Range = 8g
# MMA8452Q Control Register 1
MMA8452Q_ASLP_RATE_50 = 0x00 # Sleep mode rate = 50Hz
MMA8452Q_ASLP_RATE_12_5 = 0x40 # Sleep mode rate = 12.5Hz
MMA8452Q_ASLP_RATE_6_25 = 0x80 # Sleep mode rate = 6.25Hz
MMA8452Q_ASLP_RATE_1_56 = 0xC0 # Sleep mode rate = 1.56Hz
MMA8452Q_ODR_800 = 0x00 # Output Data Rate = 800Hz
MMA8452Q_ODR_400 = 0x08 # Output Data Rate = 400Hz
MMA8452Q_ODR_200 = 0x10 # Output Data Rate = 200Hz
MMA8452Q_ODR_100 = 0x18 # Output Data Rate = 100Hz
MMA8452Q_ODR_50 = 0x20 # Output Data Rate = 50Hz
MMA8452Q_ODR_12_5 = 0x28 # Output Data Rate = 12.5Hz
MMA8452Q_ODR_6_25 = 0x30 # Output Data Rate = 6.25Hz
MMA8452Q_ODR_1_56 = 0x38 # Output Data Rate = 1_56Hz
MMA8452Q_MODE_NORMAL = 0x00 # Normal Mode
MMA8452Q_MODE_REDUCED_NOISE = 0x04 # Reduced Noise Mode
MMA8452Q_MODE_FAST_READ = 0x02 # Fast Read Mode
MMA8452Q_MODE_ACTIVE = 0x01 # Active Mode
MMA8452Q_MODE_STANDBY = 0x00 # Standby Mode
class MMA8452Q():
def __init__(self):
self.mode_configuration()
self.data_configuration()
def mode_configuration(self):
"""Select the Control Register-1 configuration of the accelerometer from the given provided values"""
MODE_CONFIG = (MMA8452Q_ODR_800 | MMA8452Q_MODE_NORMAL | MMA8452Q_MODE_ACTIVE)
bus.write_byte_data(MMA8452Q_DEFAULT_ADDRESS, MMA8452Q_REG_CTRL_REG1, MODE_CONFIG)
def data_configuration(self):
"""Select the Data Configuration Register configuration of the accelerometer from the given provided values"""
DATA_CONFIG = (MMA8452Q_DATA_CFG_FS_2)
bus.write_byte_data(MMA8452Q_DEFAULT_ADDRESS, MMA8452Q_REG_XYZ_DATA_CFG, DATA_CONFIG)
def read_accl(self):
"""Read data back from MMA8452Q_REG_STATUS(0x00), 7 bytes
Status register, X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB"""
data = bus.read_i2c_block_data(MMA8452Q_DEFAULT_ADDRESS, MMA8452Q_REG_STATUS, 7)
# Convert the data
xAccl = (data[1] * 256 + data[2]) / 16
if xAccl > 2047 :
xAccl -= 4096
yAccl = (data[3] * 256 + data[4]) / 16
if yAccl > 2047 :
yAccl -= 4096
zAccl = (data[5] * 256 + data[6]) / 16
if zAccl > 2047 :
zAccl -= 4096
return {'x' : xAccl, 'y' : yAccl, 'z' : zAccl}
from MMA8452Q import MMA8452Q
mma8452q = MMA8452Q()
while True :
mma8452q.mode_configuration()
mma8452q.data_configuration()
time.sleep(0.5)
accl = mma8452q.read_accl()
print "Acceleration in X-Axis : %d"%(accl['x'])
print "Acceleration in Y-Axis : %d"%(accl['y'])
print "Acceleration in Z-Axis : %d"%(accl['z'])
print " ************************************* "
time.sleep(0.5)


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


(/, Motion, Detection, registers)

#define FF_MT_CFG  0x15

#define FF_MT_SRC  0x16

#define FF_MT_THS  0x17

#define FF_MT_COUNT  0x18


# In[ ]:


byte val = readRegister(WHO_AM_I);  // Read WHO_AM_I register

  if (val == 0x2A) // WHO_AM_I should always be 0x2A

  { 

    Serial.println("M is online...");

  }


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


Did you configure the High Pass Filter? In fact, if you don't use it, TRANSIENT event is always set to "High" because of the 1g gravity earth acceleration.


# In[ ]:


#define MMA8652FC_CTRL_REG1                   0x2A

#define MMA8652FC_CTRL_REG2                   0x2B

#define MMA8652FC_CTRL_REG3                   0x2C

#define MMA8652FC_CTRL_REG4                   0x2D

#define MMA8652FC_CTRL_REG5                   0x2E

#define MMA8652FC_TRANSIENT_CFG          0x1D

#define MMA8652FC_TRANSIENT_SRC          0x1E

#define MMA8652FC_TRANSIENT_THS          0x1F

#define MMA8652FC_TRANSIENT_COUNT     0x20

#define MMA8652FC_HP_FILTER_CUTOFF   0x0F

#define MMA8652FC_WHO_AM_I                    0x0D

#define MMA8652FC_WHO_AM_I_VAL           0x4A

bool MMA8652FC::init(uint8_t __u8_7BitAddress)

{

    bool __b_result=false;

    uint8_t __u8_temp=0xFF;

    _u8_I2CAddress=__u8_7BitAddress<<1;

    if(bRead(MMA8652FC_WHO_AM_I,&__u8_temp,1)) {    //Check presence of the sensor

        if(__u8_temp == MMA8652FC_WHO_AM_I_VAL) {   //Check Sensor ID answer

            if(bWrite(MMA8652FC_CTRL_REG3       ,0x02   )) //Wake from Transient interrupt

                if(bWrite(MMA8652FC_CTRL_REG4       ,0x20   )) //Enable Transient Interrupt

                    if(bWrite(MMA8652FC_CTRL_REG5       ,0x20   )) //Interrupt on INT1

                        if(bWrite(MMA8652FC_TRANSIENT_CFG   ,0x1E   )) //Enable latchedevent flag and transient detection on all 3 axis

                            if(bWrite(MMA8652FC_TRANSIENT_THS   ,0x02   )) //Set transient threshold to 0.126mg (2x0.063)

                                if(bWrite(MMA8652FC_TRANSIENT_COUNT ,0x04   )) //Set transient count

                                    if(bWrite(MMA8652FC_HP_FILTER_CUTOFF,0x03   )) //Select HPF cut-off frequency

                                        if(bWrite(MMA8652FC_CTRL_REG1       ,0x21   )) { //Set ODR to 200Hz & active mode

                                            __b_result= true;

                                        }

        }

    }

    return __b_result;

}


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


Step 1: Put the device into Standby Mode: Register 0x2A CTRL_REG1
IIC_RegWrite(0x2A, 0x18); //Set the device in 100 Hz ODR, Standby
Step 2: Set Configuration Register for Motion Detection by setting the “OR” condition OAE = 1, enabling
X, Y, and the latch
IIC_RegWrite(0x15, 0xD8)
Step 3: Threshold Setting Value for the Motion detection of > 3g
Note: The step count is 0.063g/ count
• 3g/0.063g = 47.6; //Round up to 48
IIC_RegWrite(0x17, 0x30)
Step 4: Set the debounce counter to eliminate false readings for 100 Hz sample rate with a requirement
of 100 ms timer.
Note: 100 ms/10 ms (steps) = 10 counts
IIC_RegWrite(0x18, 0x0A);
Step 7: Put the device in Active Mode

IIC_RegWrite(0x2A, 0x19);

and then I am continuously reading the Register 0x16 (FF_MT_SRC) and ( INT_SOURCE) for detecting the motion. I am trying to check 7th bit of this register which is EA bit. This bit should set when there is motion event occurs.

But my test results are as below:

Getting Read value from register as 0x1 - when the accelerometer is stable and not moving.

and Read value from register as 0x0 - when any little movement in accelerometer is there.

But this is not the expected output as my EA bit is never getting set and the value I am reading from FF_MT_SRC register is 0x1 or 0x0, no value change except these two.

 And read value of  INT_SOURCE register is 0x0 always.


# In[ ]:


First off, I would recommend to check if your I2C communication is running properly Could you please try to read the WHO_AM_I register (0x0D) and all output data registers (0x01 – 0x06) while the device is perfectly level and not moving? The MMA8452Q ID is 0x2A, the X/Y axes should read ~0g and the Z axis should read ~1g, which is +1024 at ±2g range.

 

If you are reading correct data, then you can focus on the motion detection function itself. Seems like you are using the settings from AN4070. The threshold value of 3g might be high, try to decrease it to 1.26g (0x14) for example.

 

If you read first the FF_MT_SRC register and the INT_SOURCE register after it, then it is normal that the INT_SOURCE register is 0x00 as the SRC_FF_MT bit is cleared by reading the FF_MT_SRC register.

Have you tried using an interrupt instead of polling the INT_SOURCE (FF_MT_SRC) register?  Here is my example code I used for the FXLS8471Q.

Best regards,

Tomas


# In[ ]:





# In[ ]:


What is the orientation of your board? Since the accelerometer also measures the projection of the gravity vector on sensing axes, it is possible that one of the enabled axes (X or Y) is tilted, so exceeding the threshold (312.5mg) and triggering the interrupt permanently. 

That is also why we recommend using the embedded transient detection function that triggers an interrupt when any of the enabled axes has exceeded a set acceleration threshold disregarding the static acceleration (gravity).


# In[ ]:


Hi Pallavi,

If an accelerometer is tilted, it can experience acceleration in the range of +1g to -1g, so increasing the threshold value above 1g should help. If your application requires lower threshold (“more sensitivity”), then I would recommend using the transient detection function. For more information, have a look at AN4071 or this example code.

Best regards,


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:


void FXLS8471Q_Init (void)
{
  FXLS8471Q_WriteRegister(FT_MT_THS_REG, 0x85); // Set threshold to 312.5mg (5 x 62.5mg )
  FXLS8471Q_WriteRegister(FF_MT_COUNT_REG, 0x02); // Set debounce timer period to 40ms
  FXLS8471Q_WriteRegister(FF_MT_CFG_REG, 0xD8); // Latch enabled, motion detection enabled for X and Y axis
  FXLS8471Q_WriteRegister(CTRL_REG4, 0x04); // Motion interrupt enabled
  FXLS8471Q_WriteRegister(CTRL_REG5, 0x04); // Route motion interrupt to INT1 - PTD4
              
  FXLS8471Q_WriteRegister(CTRL_REG1, 0x29); // ODR = 12.5Hz, Active mode
}


# In[ ]:


In the ISR, only the interrupt flag is cleared and the FF_MT_SRC (0x16) register is read in order to clear the SRC_FFMT flag in the INT_SOURCE register and deassert the INT1 pin, as shown on the screenshot below.

 

void PORTD_IRQHandler()
{
  PORTD_PCR4 |= PORT_PCR_ISF_MASK; // Clear the interrupt flag 
  IntSource = FXLS8471Q_ReadRegister(FF_MT_SRC_REG); // Read the FF_MT_SRC register to clear the SRC_FFMT flag in the INT_SOURCE register      
  EventCounter++;
}


# In[ ]:




