# 1- install general packages
sudo DEBIAN_FRONTEND=noninteractive apt-get -y update && sudo DEBIAN_FRONTEND=noninteractive apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev 
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install libxvidcore-dev libx264-dev
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install libgtk2.0-dev
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install libatlas-base-dev gfortran
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install python3-pip

pip install numpy
apt list python*opencv*
sudo apt -y install python3-opencv 
 # apt show python3-opencv

# 2- install virtual env
# https://raspberrypi-guide.github.io/programming/create-python-virtual-environment

# 3- install packages
pip install -r requirements.txt 
