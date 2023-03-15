#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "--> Activating conda environment"
env_name=hais-node-env

#####################  Ontario511 CAMERA LIVE  ##########################

# Download Ontario511  camera
echo && echo && echo  "--> Download Ontario511  camera LIVE $1"
python lib/Ontario511_download.py

conda deactivate 
