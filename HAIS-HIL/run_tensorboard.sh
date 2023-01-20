#!/bin/bash
clear
eval "$($(which conda) 'shell.bash' 'hook')"
conda activate ml-cpu-env
# Running Tensorboard
echo && echo "Running Tensorboard [logdir = $1]"
tensorboard --logdir $1

conda deactivate 
