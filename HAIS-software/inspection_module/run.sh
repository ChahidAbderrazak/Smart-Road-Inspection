#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
conda activate $env_name
# ################################################

# # syntax
# python syntax.py

#######################  INSPECTION FUNCTIONS ##########################

# # RPLidar_sim sensor.
# echo & echo  "Visualize RPLidar_sim measurments.py"
# python inspection_module/lib/sensors.py

# # Road conditions map
# echo & echo  "Visualize the road conditions map"
# python inspection_module/lib/inspection_map.py


# # Build and generate structures hais-database 
# echo & echo  "Build and generate structures hais-database "
# python inspection_module/lib/hais_database.py

#######################  INSPECTION MODULE ##########################
python inspection_module/main_inspection.py

# python main.py
# python syntax.py

conda deactivate 
