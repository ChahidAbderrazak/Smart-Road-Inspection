#  Highway Automated Inspection System (HAIS) node

# install the needed  packages

1. **GPS sensor:** [more details](https://gpswebshop.com/blogs/tech-support-by-os-linux/how-to-connect-an-usb-gps-receiver-with-a-linux-computer)

```
$ sudo apt install gpsd
$ sudo apt install gpsd-clients
```

2. **Camera sensor:** [more details](https://jetsonhacks.com/2022/02/02/in-practice-usb-cameras-on-jetson/)

```
$ sudo apt install v4l-utils
```
 
Here are some useful commands:
```
# List attached devices
$ v4l2-ctl --list-devices
# List all info about a given device
$ v4l2-ctl --all -d /dev/videoX
# Where X is the device number. For example:
$ v4l2-ctl --all -d /dev/video0
# List the cameras pixel formats, images sizes, frame rates
$ v4l2-ctl --list-formats-ext -d /dev/videoX
```

# wifi conenction
```
$ sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```


# Run data collection 
```
$ ./run.sh
```

# Troubleshooting sensors
1. **Device or resource busy: '/dev/ttyUSB0'**

kill the process using
```
$ sudo kill <PID>
```
To know the process ID run the following command:
```
$ sudo lsof /dev/ttyUSB0
```

2. *Permission denied: '/dev/ttyX'**

you need to allow permission to the port ``` /dev/ttyX ``` using 

```
$ sudo chmod 666 /dev/ttyX
```

# Acknowledgement

The proposed method used some other existing preprocessing packages which were adapted with/without modifications. The main ressources are cited as follows:
*  [source1](https://github.com/)
* 