#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"

# ########################   Ubuntu-CPU   ########################
conda activate hais-env
echo "Running python script"
python main_firebase.py

conda deactivate 
