#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "Activating conda environment"
env_name=hais-node-env
conda activate $env_name

# echo "# Test the All sensors"
# python lib/intelRealSense_rawScan.py

echo "# Test the 3D Camera"
python lib/intelRealSense_rawScan.py



