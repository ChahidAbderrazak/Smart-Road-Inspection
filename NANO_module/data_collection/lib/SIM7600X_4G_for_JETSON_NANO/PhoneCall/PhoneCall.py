#!/usr/bin/python

import Jetson.GPIO as GPIO
import serial
import time

ser = serial.Serial('/dev/ttyTHS1',115200)
ser.flushInput()

phone_number = '10086'
powerKey = 6
rec_buff = ''

def sendAt(command,back,timeout):
	rec_buff = ''
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	if ser.inWaiting():
		time.sleep(0.01 )
		rec_buff = ser.read(ser.inWaiting())
	if back not in rec_buff.decode():
		print(command + ' ERROR')
		print(command + ' back:\t' + rec_buff.decode())
		return 0
	else:
		print(rec_buff.decode())
		return 1

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
        sendAt('ATD'+phone_number+';','OK',1)
        time.sleep(20)
        ser.write('AT+CHUP\r\n'.encode())
        print('Call disconnected')
        powerDown(powerKey)
except keyboardInterrupt:
	if ser != None:
		ser.close()
		GPIO.cleanup()

if ser != None:
	ser.close()
	GPIO.cleanup()
