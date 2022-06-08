#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"

# ########################   Ubuntu-CPU   ########################
conda activate hais-env
echo "GET GPS data"
python lib/GPS-sensor.py

conda deactivate 
