#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-webserver-env
conda activate $env_name
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
# ------------------ run the main script  ------------------
echo && echo " #################################################" 
echo " ##           HAIS VISUALIZATION                ##" 
echo " #################################################" && echo 

uvicorn webapp:app --host 0.0.0.0 --port 8000

conda deactivate 
