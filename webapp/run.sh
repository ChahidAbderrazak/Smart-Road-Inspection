#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env
conda activate $env_name
conda activate hais-webapp-env
############################################################

######################  API FUNCTIONS ##########################

# echo && echo && echo  "--> Running HAIS functions"
# python lib/utils_hais.py

# echo && echo && echo  "--> Running HAIS webserve"
# python main.py

# ######################  MySQl FUNCTIONS ##########################
# # echo && echo && echo  "--> MySQL functions"
# python lib/sql_database.py

######################  RUN THE API SERVER ##########################

echo && echo && echo  "--> Running the web-app server"
uvicorn web-app:app --host 0.0.0.0 --port 8000

conda deactivate 
