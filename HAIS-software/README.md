#  Highway Automated Inspection System (HAIS) 
The use of different sensing technologies is pushing futher research, industrial work to improve  the highways inspections by automating the inspection process, reducing the inspection duration, and participating in increasing the safety and . 
<p align="center">
<img  src="files/hais-system.png" alt="alt text" width="512" >
 </p>
<p align="center">
Figure: Illustration of the HAIS-system
</p>


The proposed road inspection solution is based conputer vision and digital image processing technique. This algorithm   processes sensors data  and detects the potential road damages and conditions:
<p align="center">
<img  src="files/HAIS-Algorithm-Flowchart.jpg" alt="alt text" width="512" >
Figue: The road inspection algorithms flowchart
</p>


# Instructions
1. **Setup the conda environment**
```
$ conda create -n hais-node-env python=3.7 conda -y 
$ conda activate hais-node-env
$ pip install -r setup-env/requirements.txt 
```

2. **Run road inspection system**
```
./run.sh
```



# Project components
1. Inspection node design
get the sensors data and shared them on firebase
```
$ run python

```
2. Box enclosure
water-proof eclosure of the electronic devices nad inspectin node.

2. Drone control
INtegrate bridge inspection in drones with cameras

# Acknowledgement

The proposed method used some other existing preprocessing packages which were adapted with/without modifications. The main ressources are cited as follows:
*  [source1](https://github.com/)
* 