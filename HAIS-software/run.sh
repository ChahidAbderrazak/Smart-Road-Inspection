#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"
env_name=hais-node-env
conda activate $env_name
# ################################################

# # syntax
# python syntax.py

#######################  INSPECTION MODULE ##########################

# python inspection_module/main_inspection.py

# # RPLidar sensor.
# echo "Visualize RPLidar measurments.py"
# python inspection_module/lib/RPLidar-measurments.py

# Road conditions map
echo "Visualize the road conditions map"
python inspection_module/lib/inspection_map.py

#######################  INSPECTION MODULE ##########################
# python main.py
# python syntax.py

conda deactivate 
