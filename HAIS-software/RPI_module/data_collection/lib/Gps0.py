
# This program was modified and tested by Manir on 2022.09.20 and working fine

#! /usr/bin/python
import time
import smbus
import signal
import sys

BUS = None
address = 0x42
rdIntrvl = 1

lat = 190
lng = 190
alt = -1000
 

def connectBus():
    global BUS
    BUS = smbus.SMBus(1)
                     
def handle_ctrl_c(signal, frame):
        sys.exit(130)

signal.signal(signal.SIGINT, handle_ctrl_c)

def gpsDt():
    lat = 190
    lng = 190
    alt = -1000
    c = None
    response = []
    try:
        while True:
            c = BUS.read_byte(address)
            if c == 255:
 
                err=True
                lat, lng, alt= -1, -1, -1
                return err, lat, lng, alt

            elif c == 10:
             break
            else:
                response.append(c)

        if(response.count(36) == 1):
            if len(response) < 84:
                CharError = 0;
                for c in response:
                    if (c < 32 or c > 122) and  c != 13:
                        CharError+=1
                if (CharError == 0):
                    gpsChars = ''.join(chr(c) for c in response)
                    if (gpsChars.find('txbuf') == -1):
                        gpsStr, chkSum = gpsChars.split('*',2)
                        gpsComponents = gpsStr.split(',')
                        chkVal = 0
                        for ch in gpsStr[1:]:
                             chkVal ^= ord(ch)
                        if (chkVal == int(chkSum, 16)):
                            lat = 190
                            lng = 190
                            alt = -1000
                            d = 0
                            line1 = gpsChars[1:]

                            if line1[:5] == 'GNGGA':
                                d = line1.find(',')
                                d += 1
                                line1 = line1[d:]
                                d = line1.find(',')
                                d += 1
                                line1 = line1[d:]
                                d = line1.find(',')
                                lat = float(line1[:d])
                                d += 1
                                line1 = line1[d:]
                                latD = line1[:1]
                                if latD == 'S':
                                    lat = lat * (-1)
                                print(lat)
                                print(latD)
                                line1 = line1[2:]
                                d = line1.find(',')
 
                                lng = float(line1[:d])
                                d += 1
                                line1 = line1[d:]
                                lngD = line1[:1]
                                if lngD == 'W':
                                    lng = lng * (-1)
                                print(lng)
                                print(lngD)

                                line1 = line1[2:]
                                for i in range(0, 3):
                                    d = line1.find(',')
                                    d += 1
                                    line1 = line1[d:]

                                d = line1.find(',')
                                alt = float(line1[:d])
                                print(alt, "\n")
                                err=False
                                return err, lat, lng, alt

    except IOError:
        connectBus()

    except Exception as e:
        print (e)


connectBus()
