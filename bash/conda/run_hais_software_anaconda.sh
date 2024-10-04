#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
######################  RUN HAIS-project software LAPTOP ##########################
echo && echo && echo  "--> Running HAIS-dev software"
gnome-terminal  --tab --title="HAIS Desktop application" -e "sh -c 'cd src; bash run.sh '" \
                --tab --title=" HAIS Web-server" -e "sh -c 'cd webapp; bash run.sh '" 