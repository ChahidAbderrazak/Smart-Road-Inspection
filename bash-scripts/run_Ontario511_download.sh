#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env

# conda activate $env_name
echo  "--> Activating conda environment"
#####################  Ontario511 CAMERA LIVE  ##########################

# Download Ontario511  camera
echo && echo && echo  "--> Download Ontario511  camera LIVE $1"
python lib/Ontario511_download.py

# conda deactivate 
