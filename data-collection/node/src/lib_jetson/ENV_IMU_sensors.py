import time
import math
from PIL import Image,ImageDraw,ImageFont

try:
	from ENV_sensors import SH1106 #OLED
	from ENV_sensors import ICM20948 #Gyroscope/Acceleration/Magnetometer
	from ENV_sensors import BME280   #Atmospheric Pressure/Temperature and humidity
	from ENV_sensors import SI1145   #UV
	from ENV_sensors import TSL2591  #LIGHT
	# from ENV_sensors import ADS1015  #ad
except:
	from lib_jetson.ENV_sensors import SH1106 #OLED
	from lib_jetson.ENV_sensors import ICM20948 #Gyroscope/Acceleration/Magnetometer
	from lib_jetson.ENV_sensors import BME280   #Atmospheric Pressure/Temperature and humidity
	from lib_jetson.ENV_sensors import SI1145   #UV
	from lib_jetson.ENV_sensors import TSL2591  #LIGHT
	# from lib_jetson.ENV_sensors import ADS1015  #ad


bme280 = BME280.BME280()
bme280.get_calib_param()
light = TSL2591.TSL2591()
si1145 = SI1145.SI1145()
# ad = ADS1015.ADS1015()
oled = SH1106.SH1106()
icm20948 = ICM20948.ICM20948()

def get_wheather_sensors_data(LCD_visualize=False):
	if LCD_visualize:
		image = Image.new('1', (oled.width, oled.height), "BLACK")
		draw = ImageDraw.Draw(image)
		font = ImageFont.truetype('Font.ttc', 10)

	try:
		bme = []
		bme = bme280.readData()
		pressure = round(bme[0], 2) 
		temp = round(bme[1], 2) 
		hum = round(bme[2], 2)
		
		lux = round(light.Lux(), 2)
		
		uv = round(si1145.readdata()[0], 2) 
		ir = round(si1145.readdata()[1], 2)
		
		# SoundAD = []
		# Sound_normal = 3300 / 2 #1.65v
		# for i in range(10):
		# 	SoundAD.append(abs(ad.ADS1015_SINGLE_READ(3) * 2 - Sound_normal))
		# 	time.sleep(0.02)
		# SoundAD.sort()
		# SoundAD = SoundAD[1:-1]
		# Sound_ave = sum(SoundAD)/len(SoundAD)
		# Sound = Sound_ave / 50
		dict_wheather={'hPa':pressure, 'temp':temp, 'RH':hum, 'Lux':lux, 'UV':uv,'IR':ir}
		print('wheather =', dict_wheather)
		

		if LCD_visualize:
			draw.rectangle((0, 0, 128, 64), fill = 0)
			
			draw.text((0, 0), str(pressure), font = font, fill = 1)
			draw.text((40, 0), 'hPa', font = font, fill = 1)
			draw.text((0, 15), str(temp), font = font, fill = 1)
			draw.text((40, 15), 'C', font = font, fill = 1)
			draw.text((0, 30), str(hum), font = font, fill = 1)
			draw.text((40, 30), '%RH', font = font, fill = 1)
			
			draw.text((0, 45), str(lux), font = font, fill = 1)
			draw.text((40, 45), 'Lux', font = font, fill = 1)
			
			draw.text((65, 0), str(uv), font = font, fill = 1)
			draw.text((105, 0), 'UV', font = font, fill = 1)
			draw.text((65, 15), str(ir), font = font, fill = 1)
			draw.text((105, 15), 'IR', font = font, fill = 1)
			
			# draw.text((85, 45), str(Sound), font = font, fill = 1)
			# draw.text((70, 30), 'Sound', font = font, fill = 1)
			oled.display(image)

	except Exception as e:
		print('Error in reading  IMU data. \n Exception:', e)
		dict_wheather={'hPa':-1, 'temp':-10000, 'RH':-1, 'Lux':-1, 'UV':-1000,'IR':-1}

	return dict_wheather
			
def get_IMU_data(LCD_visualize=False):
	if LCD_visualize:
		image = Image.new('1', (oled.width, oled.height), "BLACK")
		draw = ImageDraw.Draw(image)
		font = ImageFont.truetype('Font.ttc', 10)
	try:
		icm = []
		icm = icm20948.getdata()
		roll = round(icm[0], 2)
		pitch = round(icm[1], 2)
		yaw = round(icm[2], 2)
		dict_Kinetic={'angular_position':[roll, pitch, yaw], 'Acc':icm[3:6], 'Gyr':icm[6:9], 'Mag':icm[9:12]}
		print('Kinetics =', dict_Kinetic)
		if LCD_visualize:
			draw.rectangle((0, 0, 128, 64), fill = 0)
			font8 = ImageFont.truetype('Font.ttc', 9)
			
			draw.text((0, 0), 'RPY', font = font8, fill = 1)
			draw.text((20, 0), str(roll), font = font8, fill = 1)
			draw.text((50, 0), str(pitch), font = font8, fill = 1)
			draw.text((90, 0), str(yaw), font = font8, fill = 1)
			
			draw.text((0, 15), 'Acc', font = font8, fill = 1)
			draw.text((20, 15), str(icm[3]), font = font8, fill = 1)
			draw.text((50, 15), str(icm[4]), font = font8, fill = 1)
			draw.text((90, 15), str(icm[5]), font = font8, fill = 1)

			draw.text((0, 30), 'Gyr', font = font8, fill = 1)
			draw.text((20, 30), str(icm[6]), font = font8, fill = 1)
			draw.text((50, 30), str(icm[7]), font = font8, fill = 1)
			draw.text((90, 30), str(icm[8]), font = font8, fill = 1)

			draw.text((0, 45), 'Mag', font = font8, fill = 1)
			draw.text((20, 45), str(icm[9]), font = font8, fill = 1)
			draw.text((50, 45), str(icm[10]), font = font8, fill = 1)
			draw.text((90, 45), str(icm[11]), font = font8, fill = 1)
			oled.display(image)
		
	except Exception as e:
		print('Error in reading  IMU data. \n Exception:',e)
		dict_Kinetic={'angular_position':[-1, -1, -1], 'Acc':[-1, -1, -1], 'Gyr':[-1, -1, -1], 'Mag':[-1, -1, -1]}

	return dict_Kinetic
			
if __name__ == "__main__":
	# get_IMU_data(LCD_visualize=False)
	get_wheather_sensors_data(LCD_visualize=False)