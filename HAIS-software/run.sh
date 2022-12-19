#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "--> Activating conda environment"
env_name=hais-node-env2
conda activate $env_name

#######################        SYNTAX      ##########################

# python syntax.py

#######################  SENSORS FUNCTIONS ##########################
# echo && echo && echo  "--> Test sensors functions"
# python lib/sensors.py

#######################  INSPECTION MODULE ##########################

# # Run the DSP Based inspection algorithm 
# echo && echo && echo  "--> Run the DSP Based inspection algorithm"
# python  lib/inspection_algorithm.py


# Run lane inspection algorithm 
echo && echo && echo  "--> Run the lane marker inspection algorithm"
python  lib/inspection_algorithm.py

#######################  VISUALIZATION MODULE ##########################

# # Road conditions map
# echo && echo && echo  "--> Visualize the road conditions map"
# python lib/inspection_map.py

#######################  DATABASE MODULE ##########################

# # Convert drone data to HAIS database stucture
# echo && echo && echo  "--> Build and generate structures HAIS-database "
# python lib/dji_drone.py

# # Build Nuscene-like database  using HAIS database stucture
# echo && echo  "--> Build Nuscene-like database  using HAIS database stucture "
# python lib/HAIS_database.py

#######################  HAIS SOFTWARE ##########################

# # run the main script
# echo && echo " #################################################" 
# echo " ##      HAIS Inspection DETECTION PROJECT      ##" 
# echo " #################################################" && echo 

# python main.py

conda deactivate 
