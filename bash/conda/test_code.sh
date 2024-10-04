#!/bin/bash
eval "$($(which conda) 'shell.bash' 'hook')"
env_name=hais-node-env
conda activate $env_name
pytest
conda deactivate 
