#!/usr/bin/python

import Jetson.GPIO as GPIO
import serial
import time

ser = serial.Serial('/dev/ttyTHS1',115200)
ser.flushInput()

powerKey = 6
rec_buff = ''
APN = 'CMNET'
ServerIP = ' 207.164.95.54' #'116.30.218.195'
Port = '5001'
Message = 'Waveshare'

def powerOn(powerKey):
	print('SIM7600X is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(powerKey,GPIO.OUT)
	time.sleep(0.1)
	GPIO.output(powerKey,GPIO.HIGH)
	time.sleep(2)
	GPIO.output(powerKey,GPIO.LOW)
	time.sleep(20)
	ser.flushInput()
	print('SIM7600X is ready')

def powerDown(powerKey):
	print('SIM7600X is loging off:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(powerKey,GPIO.OUT)
	GPIO.output(powerKey,GPIO.HIGH)
	time.sleep(3)
	GPIO.output(powerKey,GPIO.LOW)
	time.sleep(18)
	print('Good bye')
	
def sendAt(command,back,timeout):
	rec_buff = ''
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	if ser.inWaiting():
		time.sleep(0.1 )
		rec_buff = ser.read(ser.inWaiting())
	if rec_buff != '':
		if back not in rec_buff.decode():
			print(command + ' ERROR')
			print(command + ' back:\t' + rec_buff.decode())
			return 0
		else:
			print(rec_buff.decode())
			return 1
	else:
		print(command + ' no responce')

def checkStart():
        while True:
            ser.write( ('AT\r\n').encode() )
            time.sleep(0.1);
            if ser.inWaiting():
                time.sleep(0.01)
                recBuff = ser.read(ser.inWaiting())
                print( 'try to start\r\n' + recBuff.decode() )
                if 'OK' in recBuff.decode():
                    recBuff = ''
                    return
            else:
                powerOn(powerKey)
                time.sleep(1)

try:
	checkStart()
	sendAt('AT+CSQ','OK',1)
	sendAt('AT+CREG?','+CREG: 0,1',1)
	sendAt('AT+CPSI?','OK',1)
	sendAt('AT+CGREG?','+CGREG: 0,1',0.5)
	sendAt('AT+CGSOCKCONT=1,\"IP\",\"'+APN+'\"','OK',1)
	sendAt('AT+CSOCKSETPN=1', 'OK', 1)
	sendAt('AT+CIPMODE=0', 'OK', 1)
	sendAt('AT+NETOPEN', '+NETOPEN: 0',5)
	sendAt('AT+IPADDR', '+IPADDR:', 1)
	sendAt('AT+CIPOPEN=0,\"TCP\",\"'+ServerIP+'\",'+Port,'+CIPOPEN: 0,0', 5)
	sendAt('AT+CIPSEND=0,', '>', 2)#If not sure the message number,write the command like this: AT+CIPSEND=0, (end with 1A(hex))
	ser.write(Message.encode())
	if 1 == sendAt(b'\x1a'.decode(),'OK',5):
		print('send message successfully!')
	sendAt('AT+CIPCLOSE=0','+CIPCLOSE: 0,0',15)
	sendAt('AT+NETCLOSE', '+NETCLOSE: 0', 1)
	powerDown(powerKey)
except:
        if ser != None:
            ser.close()
        powerDown(powerKey)
        GPIO.cleanup()
