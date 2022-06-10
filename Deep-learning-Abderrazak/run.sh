#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"
conda activate hais-env
# ################################################

# # RPLidar sensor.
# echo "Visualize RPLidar measurments.py"
# python lib/visualize-RPLidar-measurments.py

# Road conditions map
echo "Visualize the road conditions map"
python lib/visualize-road-inspection-on-Map.py

conda deactivate 
