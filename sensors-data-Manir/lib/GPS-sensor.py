import os
# function 

def GPS_listen():
    # read data from the sensors
    time = 120
    value = 20
    return time, value


def main():
    
    # ReadGPS data
    time, value = GPS_listen()
    print(f'\n\n - time = {time},  location = {value}')

if __name__ == '__main__':
    main()


