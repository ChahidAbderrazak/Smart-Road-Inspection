#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
conda activate hais-node-env
############################################################

# echo "Running HAIS webserver"
# cd ../ 
# python webpage/main.py



echo "Running the web-app on Ubuntu"
uvicorn web-app:app --host 0.0.0.0 --port 8000


conda deactivate 
