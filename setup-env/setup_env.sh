#!/bin/bash
env_name=hais-node-env
python_version=3.7
eval "$($(which conda) 'shell.bash' 'hook')"
####################################################
clear && echo && echo " -> Setup env conda environment"
conda create -n $env_name python=$python_version conda -y

echo && echo " -> Activating conda environment"
conda activate $env_name
 
echo && echo " -> Install pip3 packages"
pip3 install -r requirements.txt 
 
echo && echo " -> Saving the version of installed pip3 packages"
pip3 freeze > requirements_versions.txt

echo && echo " -> Saving the version of installed YML packages"
conda env export > environment_droplet.yml
