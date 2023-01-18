#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
conda activate hais-node-env
############################################################

######################  API FUNCTIONS ##########################

# echo && echo && echo  "--> Running HAIS functions"
# python lib/utils_hais.py

# echo && echo && echo  "--> Running HAIS webserve"
# python main.py


######################  RUN THE API SERVER ##########################

echo && echo && echo  "--> Running the web-app server"
uvicorn web-app:app --host 0.0.0.0 --port 8000

conda deactivate 
