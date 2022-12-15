#!/usr/bin/python

import Jetson.GPIO as GPIO
import serial
import time

ser = serial.Serial("/dev/ttyTHS1",115200)
ser.flushInput()

powerKey = 6
commandInput = ''
recBuff = ''

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
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(powerKey,GPIO.OUT)
	print('SIM7600X is loging off:')
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
        while True:
            commandInput = input('Please input the AT command:')
            ser.write((commandInput + '\r\n').encode())
            time.sleep(0.1)
            if ser.inWaiting():
                    time.sleep(0.01)
                    recBuff = ser.read(ser.inWaiting())
            if recBuff != '':
                    print(recBuff.decode())
                    recBuff = ''
except:
        if ser != None:
            ser.close()
        powerDown(powerKey)
        GPIO.cleanup()
