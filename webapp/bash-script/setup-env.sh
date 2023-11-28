#!/bin/bash
env_name=hais-webserver-env
python_version=3.7
eval "$($(which conda) 'shell.bash' 'hook')"
####################################################
clear && echo && echo " -> Setup env conda environment"
conda create -n $env_name python=$python_version conda -y

echo && echo " -> Activating conda environment"
conda activate $env_name
 
echo && echo " -> Install pip3 packages"
pip3 install -r requirements.txt 

conda deactivate 