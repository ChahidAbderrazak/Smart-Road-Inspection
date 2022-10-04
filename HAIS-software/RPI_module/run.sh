#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
conda activate $env_name

# run the scripts
cd data_collection
python start_data_collection.py

