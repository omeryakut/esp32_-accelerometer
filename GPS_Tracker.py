#!/usr/bin/env python
# coding: utf-8

# In[1]:


#ESP32
  
import machine
import time
GSM_PWR = machine.Pin(4, machine.Pin.OUT)
LED = machine.Pin(12, machine.Pin.OUT)

GSM_PWR.value(1)
time.sleep_ms(300)
GSM_PWR.value(0)
LED.value(1)

gsm = machine.UART(1,tx=27, rx=26, timeout=1000,  baudrate=9600)









# In[ ]:


#SIM7000G-GSM
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


# In[ ]:


#SIM7000G-GSM2
def connect():
    from machine import UART

    GSM_PWR = machine.Pin(4, machine.Pin.OUT)
    GSM_RST = machine.Pin(5, machine.Pin.OUT)
    GSM_MODEM_PWR = machine.Pin(23, machine.Pin.OUT)

    GSM_PWR.value(0)
    GSM_RST.value(1)
    GSM_MODEM_PWR.value(1)

    GSM = UART(1, baudrate=115200, timeout=1000, rx=26, tx=27)

    GSM_APN = '[APN]'
    logline('Waiting for AT command response...')
    atcmd('AT', GSM)
    atcmd('ATZ', GSM)
    time.sleep_ms(500)
    atcmd('ate0', GSM)
    atcmd('AT+CPIN?', GSM)
    atcmd('AT+CREG?', GSM)
    atcmd('AT+CNMI=0,0,0,0,0', GSM)

    logline("Connecting to GSM...")

    atcmd('AT+CGDCONT=1,"IP","' + GSM_APN + '"', GSM)
    atcmd('AT+CGDATA="PPP",1', GSM)

    return connectGPRS(GSM)


def atcmd(cmd, GSM, wait=500, tries=20):
    GSM.write(cmd + '\r\n')
    time.sleep_ms(wait)

    for retry in range(tries):
        resp = GSM.read()
        if resp:
            print(resp)
            return resp
        else:
            log('.')
            time.sleep_ms(wait)
    else:
        raise Exception("Modem not responding!")


def connectGPRS(GSM):
    import network
    GPRS = network.PPP(GSM)
    GPRS.active(True)
    GPRS.connect()

    for retry in range(20):

        ret = GPRS.isconnected()

        if ret:
            break
        else:
            log('.')
            time.sleep_ms(2000)
    else:
        raise Exception("Modem not responding!")

    logline(GPRS.ifconfig())
    import ssltest
    return GPRS


# In[ ]:


#SIM7000G-GPS

#resp=b'\r\n+CGNSINF: 1,0,,,,,,,0,,,,,,27,,,,36,,\r\n\r\nOK\r\n'

def check_gps_data(resp):
    liste = resp.decode("utf-8").split(",")
    if liste[1]=='1':
        return 1
    else:
        return 0
    
def get_gps_data_string(resp): 
    try:
        liste = resp.decode("utf-8").split(",")
        saat_farki=3
        datetime = liste[2][6:8]+"-"+liste[2][4:6]+"-"+liste[2][0:4]+" "+str(int(liste[2][8:10])+saat_farki)+":"+liste[2][10:12]+":"+liste[2][12:14]
        if int(liste[2][8:10])+saat_farki >23:
            gun=int(liste[2][6:8])+1
            saat= int(liste[2][8:10])+saat_farki-24
            datetime = str(gun)+"-"+liste[2][4:6]+"-"+liste[2][0:4]+" "+str(saat)+":"+liste[2][10:12]+":"+liste[2][12:14]
        gps_info = "Uydu Sayısı:("+liste[15]+"/"+liste[14]+")"
        maps_link = "http://www.google.com/maps/place/"+liste[3]+","+liste[4]
        hiz=float(liste[6])* 1.852
        print(datetime)
        print(gps_info)
        print(maps_link)
        telegram_message_gps = datetime+" "+gps_info+" Hiz:"+str(hiz)+" "+maps_link
    except:
        telegram_message_gps="GPS Verisi Alınamadı."
    return telegram_message_gps


# In[2]:


#TELEGRAM

def telegram_bot_sendtext(bot_message):   
    bot_token = ''
    bot_chatID = ''
    send_text = 'https://api.telegram.org/bot' + bot_token + '/sendMessage?chat_id=' + bot_chatID + '&parse_mode=Markdown&text=' + bot_message
    response = urequests.get(send_text)
    return response.json()

def telegram_bot_receive_last_text():
    respons = urequests.get('https://api.telegram.org/botXXXXX/getUpdates')
    gelen_son_mesaj = list(respons.json().items())[0][1][len(list(respons.json().items())[0][1])-1]['message']['text']
    return gelen_son_mesaj


# In[ ]:





# In[ ]:





# In[ ]:




