
# This program was modified and tested by Manir on 2022.09.20 and working fine

import lib.Gps
import lib.Imu
import time

rdIntrvl = 0.1
isDt = 0

print("\nGPS Data:")
print()

for i in range(0, 10):
    try:
        time.sleep(rdIntrvl)
        lat, lng, alt = Gps.gpsDt()
        #gpsDt()
        print("Latitude:  ", lat)
        print("Longitude: ", lng)
        print("Altitude:  ", alt, "\n")
        #print(gpsDt)
        isDt = 1
    except Exception as e:
        pass

        #print (e)
if (isDt != 1):
    print("No valid GPS data was found\n")
    
print ("Gyroscope, Acelemeter and magnetometer data:")
print("")
try:
    AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY = Imu.imuDt()

    print("Accelaration X angle: ", AccXangle)
    print("Accelaration Y angle: ", AccYangle)
    print("Gyroscope X angle: ", gyroXangle)
    print("Gyroscope Y angle: ", gyroYangle)
    print("Gyroscope Z angle: ", gyroZangle)
    print("CF angle X: ", CFangleX)
    print("Heading: ", heading)
    print("Tilt Compensated Heading: ", tiltCompensatedHeading)
    print("Kalman X: ", kalmanX)
    print("Kalman Y: ", kalmanY)

except Exception as e:
    pass
