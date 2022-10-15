#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
conda activate $env_name

#######################  SENSORS FUNCTIONS ##########################

# # RPLidar_sim sensor.
# echo & echo  "Visualize RPLidar measurments"
# python lib/sensors.py

#######################  INSPECTION MODULE ##########################

# # Build and generate structures hais-database 
# python  lib/inspection_algorithm.py

#######################  VISUALIZATION MODULE ##########################

# # Road conditions map
# echo & echo  "Visualize the road conditions map"
# python lib/inspection_map.py

#######################  DATABASE MODULE ##########################

# # Build and generate structures hais-database 
# echo & echo  "Build and generate structures hais-database "
# python lib/hais_database.py


#######################  HAIS SOFTWARE ##########################
# python main.py
python syntax.py

conda deactivate 
