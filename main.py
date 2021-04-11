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

    
