#!/usr/bin/python

import Jetson.GPIO as GPIO
import serial
import time

ser = serial.Serial("/dev/ttyTHS1",115200)
ser.flushInput()

phone_number = '+14372564675' #********** change it to the phone number you want to call
text_message = 'BIG thank to IDREES'
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

def SendShortMessage(phone_number,text_message):
	
	print("Setting SMS mode...")
	sendAt("AT+CMGF=1","OK",1)
	print("Sending Short Message")
	answer = sendAt("AT+CMGS=\""+phone_number+"\"",">",2)
	if 1 == answer:
		ser.write(text_message.encode())
		ser.write(b'\x1A')
		answer = sendAt('','OK',20)
		if 1 == answer:
			print('send successfully')
		else:
			print('error')
	else:
		print('error%d'%answer)

def ReceiveShortMessage():
	rec_buff = ''
	print('Setting SMS mode...')
	sendAt('AT+CMGF=1','OK',1)
	sendAt('AT+CPMS=\"SM\",\"SM\",\"SM\"', 'OK', 1)
	answer = sendAt('AT+CMGR=1','+CMGR:',2)
	if 1 == answer:
		answer = 0
		if 'OK' in rec_buff:
			answer = 1
			print(rec_buff)
	else:
		print('error%d'%answer)
		return False
	return True

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
	print('Sending Short Message Test:')
	SendShortMessage(phone_number,text_message)
	print('Receive Short Message Test:\n')
	print('Please send message to phone ' + phone_number)
	ReceiveShortMessage()
	powerDown(powerKey)
except:
	if ser != None:
		ser.close()
		powerDown()
	GPIO.cleanup()
