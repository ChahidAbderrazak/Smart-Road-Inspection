#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"
env_name=hais-node-env
conda activate $env_name
# ################################################

# un the telecomuniaction of module 
 python run.py

conda deactivate 
