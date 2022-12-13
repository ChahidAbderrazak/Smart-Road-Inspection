#  Highway Automated Inspection System (HAIS) node


# install the needed  packages

1. **GPS sensor**
```
$ sudo apt install gpsd
$ sudo apt install gpsd-clients
```
[more details](https://gpswebshop.com/blogs/tech-support-by-os-linux/how-to-connect-an-usb-gps-receiver-with-a-linux-computer)


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



# Acknowledgement

The proposed method used some other existing preprocessing packages which were adapted with/without modifications. The main ressources are cited as follows:
*  [source1](https://github.com/)
* 