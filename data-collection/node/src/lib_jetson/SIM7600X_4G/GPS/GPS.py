#!/usr/bin/python

import Jetson.GPIO as GPIO
import serial
import time

ser = serial.Serial('/dev/ttyTHS1',115200)
ser.flushInput()

powerKey = 6
rec_buff = ''
rec_buff2 = ''
time_count = 0

def sendAt(command,back,timeout):
	global rec_buff
	#rec_buff = ''
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	if ser.inWaiting():
		time.sleep(0.01 )
		rec_buff = ser.read(ser.inWaiting())
	if rec_buff != '':
		if back not in rec_buff.decode():
			# print(' GPS ERROR')
			# print(command + ' ERROR')
			# print(command + ' back:\t' + rec_buff.decode())
			return 0
		else:
			# print('GPS data: ',rec_buff.decode())
			return 1
	else:
		print('GPS is not ready')
		return 0

def getGpsPosition():
	global rec_buff
	rec_null = True
	answer = 0
	# print('Start GPS session...')
	try:
		rec_buff = ''
		sendAt('AT+CGPS=1,1','OK',1)
		time.sleep(2)
	# while rec_null:
		answer = sendAt('AT+CGPSINFO','+CGPSINFO: ',1)
		#print('answer=', answer)
		#print('rec_buff=', rec_buff)
		if 1 == answer:
			answer = 0
			position=rec_buff.decode().split('+CGPSINFO: ')[1]
			position=position.split('\r')[0]

#u'4356.741364,N,07853.813160,W,270223,181044.0,161.4,0.0,
			names='Latitude, N, Longitude, W, Date, Time, Altitude, Speed, Navigation_Angle'.split(',')
			pos=position.split(',')
			# #print(f'\n GPS data: \n - metrics: \t{names} \n - values: \t{pos}')
			Latitude, N, Longitude, W, Date, Time,Altitude, Speed, Navigation_Angle=position.split(',')
			Latitude,  Longitude, Date, Time,Altitude, Speed=float(Latitude)/100.0, float(Longitude)/100.0, float(Date), float(Time), float(Altitude), float(Speed)
			car_location=[Latitude, Longitude, Altitude]

			dict_GPS={	"car_location":car_location,
						"Speed":Speed,
						"Time":Time,
						"Navigation_Angle":Navigation_Angle}
			if ',,,,,,,,' in rec_buff.decode():
				print('GPS is not ready,wait 10 seconds')
				rec_null = False
				# flag: recheck if 10 sec sleep is needed
				# time.sleep(10)
			else:
				err=False
		else:
			print('error %d'%answer)
			rec_buff = ''
			sendAt('AT+CGPS=0','OK',1)
			err=True
			dict_GPS={	"car_location":[-1, -1, -1],
						"Speed":-1000,
						"Time":-1,
						"Navigation_Angle":-1}
			time.sleep(1.5)
		# display GPS data
		return  err, dict_GPS
		
	except Exception as e:
		#print(f'Error in reading  GPS data. \n Exception: {e}')
		dict_GPS={	"car_location":[-1, -1, -1],
					"Speed":-1000,
					"Time":-1,
					"Navigation_Angle":-1}
	err=True
	return  err, dict_GPS


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
            time.sleep(0.1)
            if ser.inWaiting():
                time.sleep(0.01)
                recBuff = ser.read(ser.inWaiting())
                print( '\n - Starting GPS [ SIM7600X ] Module \r\n' + recBuff.decode() )
                if 'OK' in recBuff.decode():
                    recBuff = ''
                    return
            else:
                powerOn(powerKey)
                time.sleep(1)
def init():
	checkStart()
	getGpsPosition()
	powerDown(powerKey)

	try:
		checkStart()
		getGpsPosition()
		powerDown(powerKey)
	except:
		if ser != None:
			ser.close()
		powerDown(powerKey)
		GPIO.cleanup()
	if ser != None:
			ser.close()
			GPIO.cleanup()	

checkStart()
getGpsPosition()
