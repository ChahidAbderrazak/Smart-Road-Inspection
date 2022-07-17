#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"

# ########################   Ubuntu-CPU   ########################
conda activate nuscenes
echo "Running nuscenes python script"
python main.py 
conda deactivate 
