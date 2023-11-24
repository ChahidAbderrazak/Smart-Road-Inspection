#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env
conda activate $env_name
cd ../
######################  API FUNCTIONS ##########################

# echo && echo && echo  "--> Running HAIS functions"
# python lib/utils_hais.py

# echo && echo && echo  "--> Running HAIS web-server"
# python main.py

# echo && echo && echo  "--> Running wabpage server"
# cd webapp; bash run.sh

######################  RUN HAIS-project software LAPTOP ##########################
echo && echo && echo  "--> Running HAIS-dev software"
gnome-terminal  --tab --title=" Web-server" -e "sh -c 'cd webapp; bash run.sh '" \
                --tab --title="HAIS GUI" -e "sh -c 'cd HAIS-GUI; bash run.sh '" 


conda deactivate 
