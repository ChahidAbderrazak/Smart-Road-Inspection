#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=labeling_env #hais-labeling-env
conda activate $env_name
#################################################

#######################  IMAGE ANNOTATION MODULE ##########################
python ultimatlabeling_manager.py --data $1
conda deactivate
