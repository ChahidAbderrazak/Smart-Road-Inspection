#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo & echo "Activating conda environment"


# ###############################################
# conda activate carla-env
# echo & echo "Running Carla expereiment script"
# python main_carla.py
# conda deactivate 


###############################################
conda activate firebase-env
echo & echo "Running Firebase Data retreival script"
python main_firebase.py
conda deactivate 
