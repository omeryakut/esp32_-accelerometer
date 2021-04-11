import machine
import time

import sys
import urequests


# Power on the GSM module
GSM_PWR = machine.Pin(4, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)

GSM_PWR.value(1)
time.sleep_ms(300)
GSM_PWR.value(0)
LED.value(1)


import machine
import time
gsm = machine.UART(1,tx=27, rx=26, timeout=1000,  baudrate=9600)
time.sleep(1)
gsm.write("at\r\n")
print(gsm.readline())
print(gsm.readline())
time.sleep(1)
gsm.write("AT+CPIN?\r\n")
print(gsm.readline())
print(gsm.readline())
time.sleep(1)
#gsm.write("ate0\r\n")
#print(gsm.readline())
#print(gsm.readline())
#time.sleep(1)
gsm.write("AT+CREG?\r\n")
print(gsm.readline())
print(gsm.readline())
time.sleep(1)
gsm.write("AT+CNMI=0,0,0,0,0\r\n")
print(gsm.readline())
print(gsm.readline())
time.sleep(1)
gsm.write('AT+CGDCONT=1,"IP","globul"\r\n')
print(gsm.readline())
print(gsm.readline())
time.sleep(1)
gsm.write('AT+CGDATA="PPP",1\r\n')
time.sleep(1)
print(gsm.readline())
print(gsm.readline())

time.sleep(5)

import network
 
ppp=network.PPP(gsm)
ppp.active(True)
time.sleep(15)

print(ppp.ifconfig())
