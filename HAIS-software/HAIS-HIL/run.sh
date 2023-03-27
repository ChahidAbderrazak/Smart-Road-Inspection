
#!/bin/bash
# clear
eval "$($(which conda) 'shell.bash' 'hook')"
echo  "--> Activating conda environment"
env_name=hais-node-env
config_file=config/config.yml # config/config_ct_server.yml #


#######################  HAIS SOFTWARE ##########################
conda activate $env_name
# conda activate hais-node-env
# ------------------ run the syntac script  ------------------

python syntax.py

# ------------------ run the main script  ------------------

# echo && echo " #################################################" 
# echo " ##      HAIS Inspection DETECTION PROJECT      ##" 
# echo " #################################################" && echo 

# python main_gui.py --cfg $config_file

conda deactivate 
