from gps import *
import time

running = True
try:
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    gps_connected=True
except:
    gps_connected=False

def getPositionData(gps):
    nx = gpsd.next()
    # For a list of all supported classes and fields refer to:
    # https://gpsd.gitlab.io/gpsd/gpsd_json.html
    if nx['class'] == 'TPV':
        lat = getattr(nx, 'lat', "Unknown")
        lng = getattr(nx, 'lon', "Unknown")
        # print ("Node position: lon = " + str(lng) + ", lat = " + str(lat))
        return False, lat, lng, -1 
    print ("\n - Node position error")
    return True, "Unknown", "Unknown", "Unknown"

def get_gps_data(repeat_gps=20):
    ''' outputs [lat, lng, alt] '''
    err=True
    cnt=0
    if not gps_connected:
        print('\n - GPS error 2: Please connect the GPS  is connected and make sure you are in an open-air area')
        return []
        
    while(err):
        err, lat, lng, alt = getPositionData(gpsd)
        cnt+=1
        if cnt==repeat_gps:
            print(f'\n - GPS error 1: the GPS sesor could not get the satelite signal within {repeat_gps} trials.')
            return []
    return lat, lng, alt
    
if __name__ == "__main__":
	get_gps_data()

