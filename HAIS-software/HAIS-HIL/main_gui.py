
import os
import gc
import sys
import time
import yaml
gc.collect()
try: 
    from src.lib.global_variables import *
except: 
    from lib.global_variables import *
# Load modules
try: 
    from src.lib import HAIS
except: 
    from lib import HAIS
		
#############################	RUN the GUI	############################
from PyQt5.QtWidgets import QApplication
#%%######################### MAIN	######################
def prepare_parser(): 
	import numpy as np
	from argparse import ArgumentParser
	parser=ArgumentParser(description='Highways inspection')
	parser.add_argument(
				"--cfg", 
				default="../config/config.yml", 
				metavar="FILE", 
				help="path to config file", 
				type=str, 
		)
	return parser

def main_GUI(file=''): 
		parser=prepare_parser()
		args=parser.parse_args()
		try: 
			config_file=args.cfg
		except: 
			config_file=file
		# print(f'\n the input config file : {config_file}')
		# run the GUI
		app=QApplication(sys.argv)
		# define the GUI style
		HAIS.set_GUI_style(app)
		main_wind=HAIS.GUI_App(config_file=config_file)
		sys.exit(app.exec_())

if __name__ =='__main__': 
		main_GUI()
