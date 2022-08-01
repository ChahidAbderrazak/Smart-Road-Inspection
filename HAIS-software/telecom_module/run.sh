#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"
env_name=hais-env
conda activate $env_name
# ################################################

# un the telecomuniaction of module 
 python main.py

conda deactivate 
