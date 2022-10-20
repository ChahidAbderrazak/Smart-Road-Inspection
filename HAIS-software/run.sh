#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
conda activate $env_name

#######################        SYNTAX      ##########################

# python syntax.py

#######################  SENSORS FUNCTIONS ##########################

# python lib/sensors.py

#######################  INSPECTION MODULE ##########################

# # Build and generate structures hais-database 
# python  lib/inspection_algorithm.py

#######################  VISUALIZATION MODULE ##########################

# # Road conditions map
# echo & echo  "Visualize the road conditions map"
# python lib/inspection_map.py

#######################  DATABASE MODULE ##########################

# Convert drone data to hais database stucture
echo & echo  "Build and generate structures hais-database "
python lib/dji_drone.py

# Build Nuscene-like database  using hais database stucture
echo & echo  "Build and generate structures hais-database "
python lib/hais_database.py

#######################  HAIS SOFTWARE ##########################
# python main.py

conda deactivate 
