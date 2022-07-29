#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo "Activating conda environment"
env_name=hais-env
conda activate $env_name
# ################################################

# # RPLidar sensor.
# echo "Visualize RPLidar measurments.py"
# python lib/visualize-RPLidar-measurments.py

# # # Road conditions map
# echo "Visualize the road conditions map"
# python lib/visualize-road-inspection-on-Map.py


# # syntax
# python syntax.py

# runthe main script
python main.py

conda deactivate 
