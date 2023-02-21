#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env
conda activate $env_name
conda activate hais-node-env
echo && echo && echo  "--> Running the HAIS Godaddy server"
uvicorn app_test:app --host 0.0.0.0 --port 8000

conda deactivate 
