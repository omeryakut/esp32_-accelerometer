#!/usr/bin/env python
# coding: utf-8

# In[ ]:


[0x1D] THRESH_TAP

THRESH_TAP holds the threshold value for tap interrupts.
The data format is unsigned, so the magnitude of the tap event
is compared to THRESH_TAP. The scale factor is 62.5 mg/LSB
(i.e. 0xFF = +16 g). A zero value may result in undesirable
behavior if Tap/Double Tap interrupts are enabled. 



[0x21] DUR

DUR is an unsigned time value representing the maximum
time that an event must be above the THRESH_TAP threshold
to qualify as a tap event. The scale factor is 625 µs/LSB. A zero
value will prevent Tap/Double Tap functions from working.



[0x22] LATENT

LATENT is an unsigned time value representing the wait time
from the detection of a tap event to the opening of the time
window WINDOW for a possible second tap event. The scale
factor is 1.25 ms/LSB. A zero value will disable the Double Tap
function.





[0x23] WINDOW

WINDOW is an unsigned time value representing the amount
of time after the expiration of LATENT during which a second
tap can begin. The scale factor is 1.25 ms/LSB. A zero value will
disable the Double Tap function.




[0x24] THRESH_ACT

THRESH_ACT holds the threshold value for activity detection.
The data format is unsigned, so the magnitude of the activity
event is compared to THRESH_ACT. The scale factor is
62.5 mg/LSB. A zero value may result in undesirable behavior if
Activity interrupt is enabled.



[0x25] THRESH_INACT

THRESH_INACT holds the threshold value for inactivity
detection. The data format is unsigned, so the magnitude of the
inactivity event is compared to THRESH_INACT. The scale
factor is 62.5 mg/LSB. A zero value may result in undesirable
behavior if Inactivity interrupt is enabled. 




[0x26] TIME_INACT

TIME_INACT is an unsigned time value representing the
amount of time that acceleration must be below the value in
THRESH_INACT for inactivity to be declared. The scale factor
is 1 second/LSB. Unlike the other interrupt functions, which
operate on unfiltered data(See Threshold description in
Application section), the inactivity function operates on the
filtered output data. At least one output sample must be
generated for the inactivity interrupt to be triggered. This will
result in the function appearing un-responsive if the
TIME_INACT register is set with a value less than the time
constant of the Output Data Rate. A zero value will result in an
interrupt when the output data is below THRESH_INACT.



[0x27] ACT_INACT_CTL

X/Y/Z Enable: A ‘1’ enables X, Y, or Z participation in activity
or inactivity detection. A ‘0’ excludes the selected axis from
participation. If all of the axes are excluded, the function is
disabled.
AC/DC: A ‘0’ = DC coupled operation and a ‘1’ = AC coupled
operation. In DC coupled operation, the current acceleration is
compared with THRESH_ACT and THRESH_INACT directly
to determine whether activity or inactivity is detected. In AC
coupled operation for activity detection, the acceleration value
at the start of activity detection is taken as a reference value.
New samples of acceleration are then compared to this
reference value and if the magnitude of the difference exceeds
THRESH_ACT the device will trigger an activity interrupt. In
AC coupled operation for inactivity detection, a reference value
is used again for comparison and is updated whenever the
device exceeds the inactivity threshold. Once the reference
value is selected, the device compares the magnitude of the
difference between the reference value and the current
acceleration with THRESH_INACT. If the difference is below
THRESH_INACT for a total of TIME_INACT, the device is
considered inactive and the inactivity interrupt is triggered.



[0x2A] TAP_AXES

TAP_X/Y/Z Enable: A ‘1’ in TAP_X, Y, or Z Enable enables X,
Y, or Z participation in Tap detection. A ‘0’ excludes the
selected axis from participation in Tap detection.
Setting the SUPPRESS bit suppresses Double Tap detection if
acceleration greater than THRESH_TAP is present between
taps. See Tap Detection in the Application Section for more
details. 



[0x2C] BW_RATE

LOW_POWER: A ‘0’ = Normal operation and a ‘1’ = Reduced
power operation with somewhat higher noise. (See Power
Modes section for details).
RATE: Selects device bandwidth and output data rate. See Table
5 and Table 6 for details. Default value is 0x0A, or 100 Hz
Output Data Rate. An Output Data Rate should be selected that
is appropriate for the communication protocol and frequency
selected. Selecting too high of an Output Data Rate with a low
communication speed will result in samples being discarded.



[0x2D] POWER_CTL

LINK: A ‘1’ with both the activity and inactivity functions
enabled will delay the start of the activity function until
inactivity is detected. Once activity is detected, inactivity
detection will begin and prevent the detection of activity. This
bit serially links the activity and inactivity functions. When ‘0’
the inactivity and activity functions are concurrent. Additional
information can be found in the Application section under Link
Mode.
AUTO_SLEEP: A ‘1’ sets the ADXL345 to switch to Sleep Mode
when inactivity (acceleration has been below THRESH_INACT
for at least TIME_INACT) is detected and the LINK bit is set.
A ‘0’ disables automatic switching to Sleep Mode. See SLEEP
for further description.
MEASURE: A ‘0’ places the part into standby mode and a ‘1’
places the part into measurement mode. The ADXL345 powers
up in standby mode with minimum power consumption.
SLEEP: A ‘0’ puts the part into a normal mode of operation. A
‘1’ places the part into Sleep Mode. This suppresses
DATA_READY, stops sending data to the FIFO, and switches
the sampling rate to one specified by the WAKEUP bits. In
Sleep Mode, only the Activity function can be used.
When clearing the LINK, AUTO_SLEEP, or SLEEP bits, it is
recommended that the part be placed into Standby when
clearing the bits and then re-enabling Measurement mode
during a following write. This is done to ensure the device is
properly biased if Sleep mode is manually disabled. Not
toggling Measurement mode may result in the first few after
LINK, AUTO_SLEEP, or SLEEP is cleared having additional
noise, especially if the device was asleep when the bits were
cleared.
WAKEUP: Controls the frequency of readings in Sleep Mode as
shown in 



[0x2E] INT_ENABLE

Setting bits with a value of ‘1’ in this register to enable their
respective functions and generate interrupts. A value of ‘0’ will
prevent the functions from generating an interrupt.
DATA_READY, WATERMARK, and OVERRUN bits only
enable the interrupt output; the functions are always enabled. It
is recommended that interrupts be configured before enabling
their outputs. 




[0x2F] INT_MAP

Any ‘0’ bits in this register send their respective interrupts to
the INT1 pin. Bits set with a ‘1’ send their respective interrupts
to the INT2 pin. All selected interrupts for a given pin are
ORed. 



[0x31] DATA_FORMAT

DATA_FORMAT controls the presentation of data at registers
0x32 to 0x37. All data, except ±16 g range, must be clipped to
avoid rollover.
SELF_TEST: A ‘1’ applies a Self Test force to the sensor causing
a shift in the output data. A value of ‘0’ disable Self Test. 


SPI: A value of ‘1’ sets the device to 3-wire SPI and a value of ‘0’
sets the device to 4-wire SPI.
INT_INVERT: A value of ‘0’ sets the interrupts to Active High
while a value of ‘1’ sets the interrupts to Active Low.
FULL_RES: When this bit is set with a value of ‘1’ the device is
in Full-Resolution Mode, where the output resolution increases
with RANGE to maintain a 4 mg/LSB scale factor. When this
bit is ’0’ the device is in 10-bit Mode and RANGE determine the
maximum g-Range and scale factor.
JUSTIFY: A ‘1’ = Left (MSB) justified and a ‘0’ = Right justified
with sign extension.
RANGE: Sets the g-Range based on Table 14 below. 


D1 D0 g-Range
0 0 ±2 g
0 1 ±4 g
1 0 ±8 g
1 1 ±16 g



THRESHOLD
The lower Output Data Rates are achieved by decimation of a
common sampling frequency inside the device. The activity,
free-fall and tap/double tap detection functions are performed
using the un-filtered data. Since the output data is filtered, the
high frequency and high-g data that is used to determine
activity, free-fall and tap/double tap events may not be present if
the output of the accelerometer is examined. This may result in
trigger events appearing to occur when acceleration does not
appear to trigger an event, such as exceeding a threshold or
remaining below a threshold for a certain period of time.


LINK MODE
The LINK function can be used to reduce the number of
activity interrupts the processor must service by only looking
for activity after inactivity. For proper operation of the link
feature, the processor must still respond to the activity and
inactivity interrupts by reading the INT_SOURCE register to
clear them. If the activity interrupt is not cleared, the part will
not go into Auto Sleep Mode. The ASLEEP bit in the
ACT_TAP_STATUS register indicates if the part is in Auto
Sleep Mode. 


# In[1]:


(/, Values, for, Activity, and, Inactivity, detection)
 accelerometer.setActivityThreshold(2.0);    // Recommended 2 g
 accelerometer.setInactivityThreshold(2.0);  // Recommended 2 g
 accelerometer.setTimeInactivity(5);         // Recommended 5 s

 (/, Set, activity, detection, only, on, X,Y,Z-Axis)
 accelerometer.setActivityXYZ(1);         // Check activity on X,Y,Z-Axis
 (/, or)
 (/, accelerometer.setActivityX(1);, //, Check, activity, on, X_Axis)
 (/, accelerometer.setActivityY(1);, //, Check, activity, on, Y-Axis)
 (/, accelerometer.setActivityZ(1);, //, Check, activity, on, Z-Axis)

 (/, Set, inactivity, detection, only, on, X,Y,Z-Axis)
 accelerometer.setInactivityXYZ(1);       // Check inactivity on X,Y,Z-Axis
 (/, or)
 (/, accelerometer.setInactivityX(1);, //, Check, inactivity, on, X_Axis)
 (/, accelerometer.setInactivityY(1);, //, Check, inactivity, on, Y-Axis)
 (/, accelerometer.setInactivityZ(1);, //, Check, inactivity, on, Z-Axis)

 (/, Select, INT, 1, for, get, activities)
 accelerometer.useInterrupt(ADXL345_INT1);


# In[ ]:


#define NOMOVEMENT_THRESHOLD 0x03 //62.5mg/LSB, 0x03=0.1875g
#define NOMOVEMENT_TIME 0x0A //1s/LSB, 0x0A=10s


# In[ ]:


1. write 0x0B to register 0x31 (DATA_FORMAT)     //choose full resolution mode, 3.9mg/LSB

2. write 0x0B to register 0x2C (BW_RATE)            //set ODR to 200Hz as you did

3. write 0x04 to reigster 0x24 (THRESH_ACT)        //set activity threshold as you mentioned

4. write 0x02 to reigster 0x25 (THRESH_INACT)     //set inactivity threshold as you mentioned

5. write 0x02 to reigster 0x26 (TIME_INACT)          //set inactivity time as you mentioned

6. write 0xff to register 0x27 (ACT_INACT_CTL)     //enable X, Y, Z in AC mode

7. write 0x18 to register 0x2E (INT_ENABLE)         //enable Activity and inactivity interrupt.

8. configure the 0x2F (INT_MAP) register according to your schematics.

9. write 0x38/0x28/0x08 to regiser 0x2D (POWER_CTL)     //enable measurment.


# In[ ]:



 (/, interrupts, setup)
 writeTo(DEVICE, R_INT_MAP, 0); // send all interrupts to ADXL345's INT1 pin
 writeTo(DEVICE, R_INT_ENABLE, B8(1111100)); // enable signle and double tap, activity, inactivity and free fall detection
 
 
 (/, free, fall, configuration)
 writeTo(DEVICE, R_TIME_FF, 0x14); // set free fall time
 writeTo(DEVICE, R_THRESH_FF, 0x05); // set free fall threshold
 
 (/, single, tap, configuration)
 writeTo(DEVICE, R_DUR, 0x1F); // 625us/LSB
 writeTo(DEVICE, R_THRESH_TAP, 48); // 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
 writeTo(DEVICE, R_TAP_AXES, B8(111)); // enable tap detection on x,y,z axes

 (/, double, tap, configuration)
 writeTo(DEVICE, R_LATENT, 0x10);
 writeTo(DEVICE, R_WINDOW, 0xFF);
 
 (/, inactivity, configuration)
 writeTo(DEVICE, R_TIME_INACT, 10); // 1s / LSB
 writeTo(DEVICE, R_THRESH_INACT, 3); // 62.5mg / LSB
 (/, also, working, good, with, high, movements:, R_TIME_INACT=5,, R_THRESH_INACT=16,, R_ACT_INACT_CTL=B8(00000111))
 (/, but, unusable, for, a, quite, slow, movements)
 
 (/, activity, configuration)
 writeTo(DEVICE, R_THRESH_ACT, 8); // 62.5mg / LSB
 
 (/, activity, and, inctivity, control)
 writeTo(DEVICE, R_ACT_INACT_CTL, B8(11111111)); // enable activity and inactivity detection on x,y,z using ac
 
 (/, set, the, ADXL345, in, measurement, and, sleep, Mode:, this, will, save, power, while, while, we, will, still, be, able, to, detect, activity)
 (/, set, the, Link, bit, to, 1, so, that, the, activity, and, inactivity, functions, aren't, concurrent, but, alternatively, activated)
 (/, set, the, AUTO_SLEEP, bit, to, 1, so, that, the, device, automatically, goes, to, sleep, when, it, detects, inactivity)
 writeTo(DEVICE, R_POWER_CTL, B8(111100));


# In[ ]:





# In[ ]:





# In[ ]:


(/set, activity/, inactivity, thresholds, (0-255))
adxl.setActivityThreshold(75); //62.5mg per increment
adxl.setInactivityThreshold(75); //62.5mg per increment
get_ipython().set_next_input('adxl.setTimeInactivity(10); // how many seconds of no activity is inactive');get_ipython().run_line_magic('pinfo', 'inactive')
(/look, of, activity, movement, on, this, axes, -, 1, ==, on;, 0, ==, off)
adxl.setActivityX(1);
adxl.setActivityY(1);
adxl.setActivityZ(1);

(/look, of, inactivity, movement, on, this, axes, -, 1, ==, on;, 0, ==, off)
adxl.setInactivityX(1);
adxl.setInactivityY(1);
adxl.setInactivityZ(1);

(/look, of, tap, movement, on, this, axes, -, 1, ==, on;, 0, ==, off)
adxl.setTapDetectionOnX(0);
adxl.setTapDetectionOnY(0);
adxl.setTapDetectionOnZ(1);

(/set, values, for, what, is, a, tap,, and, what, is, a, double, tap, (0-255))
adxl.setTapThreshold(50); //62.5mg per increment
adxl.setTapDuration(15); //625μs per increment
adxl.setDoubleTapLatency(80); //1.25ms per increment
adxl.setDoubleTapWindow(200); //1.25ms per increment

(/set, values, for, what, is, considered, freefall, (0-255))
adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment

(/setting, all, interupts, to, take, place, on, int, pin, 1)
(/I, had, issues, with, int, pin, 2,, was, unable, to, reset, it)
adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,  ADXL345_INT1_PIN );
adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,  ADXL345_INT1_PIN );
adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,  ADXL345_INT1_PIN );
adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );
adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,  ADXL345_INT1_PIN );

(/register, interupt, actions, -, 1, ==, on;, 0, ==, off)
adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT, 1);
adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,  1);
adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}


# In[ ]:


https://github.com/Giogia/Abbot/tree/266663cc79be46e0f85b4f6bbcd27b5a09de72e4/adxl345


# In[10]:


# ADXL345 constants
EARTH_GRAVITY_MS2   = 9.80665
SCALE_MULTIPLIER    = 0.004

DATA_FORMAT         = 0x31
BW_RATE             = 0x2C
POWER_CTL           = 0x2D
THRESH_ACT          = 0x24
ACT_INACT_CTL       = 0x27
INT_MAP             = 0x2F
INT_ENABLE          = 0x2E
INT_SOURCE          = 0x30


BW_RATE_1600HZ      = 0x0F
BW_RATE_800HZ       = 0x0E
BW_RATE_400HZ       = 0x0D
BW_RATE_200HZ       = 0x0C
BW_RATE_100HZ       = 0x0B
BW_RATE_50HZ        = 0x0A
BW_RATE_25HZ        = 0x09

RANGE_2G            = 0x00
RANGE_4G            = 0x01
RANGE_8G            = 0x02
RANGE_16G           = 0x03

MEASURE             = 0x08
AXES_DATA           = 0x32
INT_THRESHOLD       = 0x05
ACT_ENABLE          = 0x10
ACT_AXES            = 0xF0

adxl_address = 0x53
        


def enableInterrupt():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(GPIO_INTERRUPT_PIN, GPIO.IN)
    GPIO.add_event_detect(GPIO_INTERRUPT_PIN, GPIO.RISING, callback = handleInterrupt)

    i2c.write_byte_data(address, THRESH_ACT, INT_THRESHOLD)
    i2c.write_byte_data(address, ACT_INACT_CTL, ACT_AXES)
    i2c.write_byte_data(address, INT_MAP, 0x00)
    i2c.write_byte_data(address, INT_ENABLE, ACT_ENABLE)
    clearInterrupt()

def clearInterrupt():
    i2c.read_byte_data(address, INT_SOURCE)
    
def enableMeasurement():
    i2c.write_byte_data(address, POWER_CTL, MEASURE)
    
def setRange( range_flag):
    value = i2c.read_byte_data(address, DATA_FORMAT)

    value &= ~0x0F;
    value |= range_flag;  
    value |= 0x08;

    i2c.write_byte_data(address, DATA_FORMAT, value)
    
def setBandwidthRate( rate_flag):
    i2c.write_byte_data(address, BW_RATE, rate_flag)
    
    
    
address = 0x53
setBandwidthRate(BW_RATE_100HZ)
setRange(RANGE_2G)
enableMeasurement()
enableInterrupt()


# In[ ]:




