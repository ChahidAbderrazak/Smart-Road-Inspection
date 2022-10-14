#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
# conda activate $env_name
# ################################################

#######################  IMAGE ANNOTATION MODULE ##########################
python ultimatlabeling_manager.py

conda deactivate 
