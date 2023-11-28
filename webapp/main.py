
import os
import yaml
from argparse import ArgumentParser

def prepare_parser():
		parser = ArgumentParser(description='HAIS project visualization')
		parser.add_argument(
				"--cfg",
				default="config/config.yml",
				metavar="FILE",
				help="path to config file",
				type=str,
		)
		return parser

def get_config_parameters():
		parser = prepare_parser()
		args =	parser.parse_args()
		# Read config parameters from the sYAML file
		with open(args.cfg, 'r') as stream:
				config = yaml.safe_load(stream)
		return config

#################################################
# load the config file
config =get_config_parameters()
print(config['DATABASE']['local'])

#################################
# from markupsafe import escape
from nuscenes.nuscenes import NuScenes
from flask import Flask, render_template, request
from lib import utils, HAIS_project
import os, json 

# nitialize the flask server and download the database structure
app = Flask(__name__, template_folder='./')
download_path=os.path.join(os.getcwd(),'download')
HAIS_project.get_data()

@app.route('/download/<string:node_name>/', methods=['GET','POST'])
def download(node_name):# node name 
	if request.method == 'POST': # POST request
			print(request.get_text())	# parse as text
			return 'OK', 200

	else: # GET request
		print(os.path.join(download_path,node_name))
		if 'info.json' not in os.listdir(os.path.join(download_path,node_name)):
				HAIS_project.decommpress(node_name)
				return ({"status":'done'})
		else:
				return ({"status":'the file alrady there'})
        
@app.route('/get_routes/<string:node_name>', methods=['GET','POST'])
def data_get(node_name):
		if request.method == 'POST': # POST request
			print(request.get_text())	# parse as text
			return 'OK', 200
		
		else: # GET request
			f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
			data=json.load(f)
			return data

@app.route('/get_routes/<string:node_name>/', methods=['GET','POST'])
def  get_routes(node_name):
	if request.method == 'POST': # POST request
		print(request.get_text())	# parse as text
		return 'OK', 200
	else: # GET request
		f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
		data=json.load(f)
		return data

@app.route('/get_data/<string:node_name>/<string:token>', methods=['GET','POST'])
def get_node_data(node_name,token):
	if request.method == 'POST': # POST request
		print(request.get_text())	# parse as text
		return 'OK', 200
	else: # GET request
		dataroot=os.path.join(download_path,node_name)
		version='v1.0'
		nusc = NuScenes(version=version, dataroot=dataroot, verbose=True)
		res = utils.get_sensors_data(nusc, token)
		print(f'\n sensors data: {res}')
		return res

@app.route('/')
def index():
    return render_template('syntax.html')     

if __name__ == '__main__':
		app.run(host='localhost', port=8080)#change the host