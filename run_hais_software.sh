#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env
conda activate $env_name

######################  RUN HAIS-project software LAPTOP ##########################
echo && echo && echo  "--> Running HAIS-dev software"
gnome-terminal  --tab --title=" Web-server" -e "sh -c 'cd webpage; bash run.sh '" \
                --tab --title="HAIS GUI" -e "sh -c 'cd HAIS-Inspection; bash run.sh '" 

conda deactivate 
