import os, sys
import gc
from tkinter import E
import cv2
import time
import threading
import numpy as np
import pandas as pd
from glob import glob
from pathlib import Path
from numpy.core.records import array
import matplotlib.pyplot as plt
from pyrsistent import s

try: 
	from src.lib.global_variables import *
	from src.lib import utils, hais_database, inspection_algorithm, inspection_map
except: 
	from lib.global_variables import *
	from lib import utils, hais_database, inspection_algorithm, inspection_map
	
#%%############################	RUN the GUI	############################
import napari
from PyQt5 import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QGroupBox, QFormLayout, QLabel, QMessageBox, \
														QLineEdit, QFileDialog, QPushButton, QDialog, QCheckBox, QVBoxLayout,\
														QLabel, QSizePolicy, QScrollArea, QMessageBox, QComboBox
# from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon, QFont, QPalette, QColor, QImage, QPixmap, QPalette, QPainter
from PyQt5.QtCore import * #QObject, pyqtSignal, QThread
from PyQt5.QtPrintSupport import QPrintDialog, QPrinter

# set the transformation
import torchvision.transforms as transforms

class Worker(QObject):
	finished = pyqtSignal()
	progress = pyqtSignal(int)
	retrieved = pyqtSignal(object)
	N=0
	def run(self):
			"""run thread"""
			for i in range(self.N):
					time.sleep(0.1)
					self.progress.emit(i)
			self.finished.emit()

class HAIS_GUI(QWidget): 
	def __init__(self, config_file, disp=0): 
			super(HAIS_GUI, self).__init__()
			self.config_file=config_file
			self.config=self.load_config(config_file)
			self.root_folder=self.config['DATA']['data_root']
			self.version=self.config['DATA']['version']
			self.dst_directory=self.config['OUTPUTS']['dst_directory']
			self.maps_root=os.path.join(self.dst_directory, "maps")
			self.annotation_directory=self.config['ANNOTATION']['annotation_directory']
			# create the output folders:
			self.create_output_directries()

			# data attributes
			self.image_path=None
			self.mask_path=None
			self.file_index=0
			self.node_dict={}
			self.disp=disp

			# intialize the GUI
			self.init_GUI()
			self.display()
			
	def create_output_directries(self):
		for folder in [self.maps_root, self.annotation_directory]:
			utils.create_new_directory(folder)

	def init_GUI(self): 
			## the form attributs.
			self.tool_name=QLineEdit()
			self.tool_location=QLineEdit()
			self.data_source=QCheckBox("folders")
			self.data_source.setChecked(False)	 
			self.data_source.stateChanged.connect(lambda: self.btnstate(self.data_source))
			self.input_data_button=QPushButton('Click')# browse file
			self.input_data_button.clicked.connect(self.get_image_path)

			# Run data inspection 
			self.button_run_frame_inspection=QPushButton('Run frame by frame inspection') 
			self.button_run_frame_inspection.setFont(QFont("Times", 10, QFont.Bold))
			self.button_run_frame_inspection.clicked.connect(self.inspect_frame_by_frame)
			self.button_run_inspection=QPushButton('Run all frames inspection') 
			self.button_run_inspection.setFont(QFont("Times", 10, QFont.Bold))
			self.button_run_inspection.clicked.connect(self.inspection_thread)
			
			# visualize the inspectin and map 
			self.button_visualize=QPushButton('Show the inspection map') 
			self.button_visualize.setFont(QFont("Times", 10, QFont.Bold))
			self.button_visualize.clicked.connect(self.show_inspection_map)
			self.show_all_nodes=QCheckBox("all nodes")
			self.forcasting=QCheckBox("forcasting")
			self.show_lanemarker=QCheckBox("lanemarker")
			self.show_road_condition=QCheckBox("road conditon")

			# Annotation verification
			self.button_inteactive_annotation=QPushButton('Verify the annotation mask') 
			self.button_inteactive_annotation.setFont(QFont("Times", 10, QFont.Bold))
			self.button_inteactive_annotation.clicked.connect(self.napari_mask_verification)

			# list of nodes
			self.list_nodes_combo = QComboBox(self)
			self.list_nodes_combo.clear()
			self.list_nodes_combo.addItem("select a node")
			self.list_nodes_combo.activated.connect(self.on_node_selection)

			# list of missions
			self.list_missions_combo = QComboBox(self)
			self.list_missions_combo.clear()
			self.list_missions_combo.addItem("select a mission")
			self.list_missions_combo.activated.connect(self.on_mission_selection)

			# list of missions
			self.list_sensors_combo = QComboBox(self)
			self.list_sensors_combo.clear()
			self.list_sensors_combo.addItem("select a sensor")
			self.list_sensors_combo.activated.connect(self.on_sensor_selection)
 			
			# image viewer
			self.printer = QPrinter()
			self.scaleFactor = 0.0
			self.image_ID = QLabel()
			self.imageLabel = QLabel()
			
			self.imageLabel.setBackgroundRole(QPalette.Base)
			self.imageLabel.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
			self.imageLabel.setScaledContents(True)
			self.imageLabel.resize(resize[0], resize[1])

			self.scrollArea = QScrollArea()
			self.scrollArea.setBackgroundRole(QPalette.Dark)
			self.scrollArea.setWidget(self.imageLabel)
			self.scrollArea.setVisible(True)

			# create the formGroupBoxs .
			self.create_database_formGroupBox()
			self.create_nodes_formGroupBox()
			self.nodes_formGroupBox.setVisible(False)
			self.setVisible_inspection_widgets(False)

			self.create_visualization_formGroupBox()
			self.visualization_formGroupBox.setVisible(False)

			self.create_annotation_formGroupBox()
			self.annotation_formGroupBox.setVisible(False)

			# Compile the maun GUI layout
			mainLayout=QVBoxLayout()
			mainLayout.addWidget(self.database_formGroupBox)
			mainLayout.addWidget(self.nodes_formGroupBox)
			mainLayout.addWidget(self.image_ID)
			# mainLayout.addWidget(self.imageLabel)
			mainLayout.addWidget(self.scrollArea)
			mainLayout.addWidget(self.button_run_frame_inspection)
			mainLayout.addWidget(self.button_run_inspection)
			mainLayout.addWidget(self.visualization_formGroupBox)
			mainLayout.addWidget(self.annotation_formGroupBox)
			self.setLayout(mainLayout)
			self.setWindowTitle("Browse the collected data")
			self.setWindowTitle('HAIS-Inspection')
			self.setWindowIcon(QIcon('files/icon_hais.png'))
			self.setGeometry(100, 100, 640, 100)
			# self.setWindowFlags(
			# 		Qt.WindowCloseButtonHint
			# )
			self.show()

#%%############## GUI Routines ######################
	def setVisible_inspection_widgets(self, enable):
		self.scrollArea.setVisible(enable)
		# self.imageLabel.setVisible(enable)
		self.image_ID.setVisible(enable)
		self.button_run_inspection.setVisible(enable)

	def display(self): 
		print(f'\n\n##################### 	Running GUI based inspection	########################')
		print(f'## - data root : {self.root_folder} ')
		try: 
			print(f'## - Automated annotation destination root : {self.annotation_directory} ')
		except: 
			err=1
		print(f'################################################################################\n\n')
	
	def on_node_selection(self):
		self.node_name=str(self.list_nodes_combo.currentText())
		if self.node_name=='select a node':
			return
			
		self.mission_dict=self.node_dict[self.node_name]
		# print(f'\n - node_dict={self.node_dict }\n - node_name={self.node_name} \n - missions={self.mission_dict}')

		list_missions=list(self.mission_dict.keys())

		# print(f'\n flag: \n - list_missions={list_missions}')
		self.list_missions_combo.clear()
		self.list_missions_combo.addItem("select a mission")
		self.list_missions_combo.addItems(list_missions)

	def on_mission_selection(self):
		self.mission_name=str(self.list_missions_combo.currentText())
		if self.mission_name=='select a mission':
			return

		self.dataroot=self.mission_dict[self.mission_name]
		# print(f'\n - dataroot={self.dataroot}')
		list_sensors=[os.path.basename(path) for path in glob(os.path.join(self.dataroot, 'sweeps', '*')) if os.path.isdir(path)]
		# print(f'\n flag: \n - list_sensors={list_sensors}')
		self.list_sensors_combo.clear()
		self.list_sensors_combo.addItem("select a sensor")
		self.list_sensors_combo.addItems(list_sensors)

	def on_sensor_selection(self):
		# print(f'\n - dataroot={self.dataroot}')
		self.sensor_name=str(self.list_sensors_combo.currentText())
		if self.node_name=='select a sensor':
			return
		self.sensor_folder=os.path.join(self.dataroot, 'sweeps', self.sensor_name)
		self.list_files=[path for path in glob(os.path.join(self.sensor_folder, '*')) if os.path.isfile(path)]

		if len(self.list_files)>0:
			print(f'\n - {self.sensor_name} sensor has {len(self.list_files)} files.')
			# self.inspection_formGroupBox.setVisible(True)
			self.setVisible_inspection_widgets(True)
			self.visualization_formGroupBox.setVisible(True)
			self.annotation_formGroupBox.setVisible(False)
			# show the first file
			self.file_index=0
			self.image_path=self.list_files[self.file_index]
			self.visualize_image(self.image_path)
		else:
			msg=f'\n - No files are found in the database: \n - sensor={self.sensor_name} \n - folder ={self.sensor_folder}'
			QMessageBox.information(self, "Empty data folder Viewer", msg)

	def inspect_frame_by_frame(self):
		try:
			# Run the inspection algorithm on one image
			self.inspection_image(self.image_path)

			# get the next image path
			self.file_index+=1
			self.image_path= self.list_files[self.file_index]
		except Exception as e:
			QMessageBox.warning(self, "Error", e)
			self.file_index=0

	def show_inspection_map(self):
		if self.show_all_nodes.isChecked(): # show all nodes
			list_missions=[]
			for node_name in self.node_dict.keys():
				mission_dict=self.node_dict[node_name]
				for mission_name in mission_dict.keys():
					list_missions.append(mission_dict[mission_name])
		else:# show only the current node
			list_missions=[self.mission_dict[mission_name] for mission_name in self.mission_dict.keys()]

		# save the visualized map
		inspection_map.visualize_map(list_missions, maps_root=self.maps_root)

	def visualize_image(self, input_data):
		try:
			if isinstance(input_data, str):
				image = cv2.imread(input_data)
				filename=input_data
			else:
				image=input_data
		
			height, width, channel = image.shape
			bytesPerLine = 3 * width
			image = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
			filename='[CV2 image]'

			# print(f'\n - showing the file in the viewer: \n - {input_data}')
			self.imageLabel.setPixmap(QPixmap.fromImage(image))
			self.scaleFactor = 1.0
			return 
		except Exception as e:
			QMessageBox.warning(self, "Image Viewer", "Cannot view the data %s." % filename)
			print(f'Exception; {e}')
			return

	def load_config(self, config_file): 
		import yaml
		try: 
			# Read config parameters from the sYAML file
			with open(config_file, 'r') as stream: 
				config=yaml.safe_load(stream)
			return config
		except: 
			msg=f'\n\n Error: The config file [{config_file}] cannot be read correctly OR is not found!!'
			raise Exception(msg)

	def btnstate(self, b): 	 
			if b.isChecked()==True: 
					b.setText('files')
			else: 
					b.setText('folders')

	def create_database_formGroupBox(self): 
		self.database_formGroupBox=QGroupBox("Database setting")
		layout=QFormLayout()
		layout.addRow(QLabel("data type: "), self.data_source)
		layout.addRow(QLabel("data folder: "), self.input_data_button)
		self.database_formGroupBox.setLayout(layout)

	def create_nodes_formGroupBox(self): 
		self.nodes_formGroupBox=QGroupBox("Nodes and missions")
		layout=QFormLayout()
		layout.addRow(QLabel("Nodes: "), self.list_nodes_combo)
		layout.addRow(QLabel("Missions: "), self.list_missions_combo)
		layout.addRow(QLabel("Sensor: "), self.list_sensors_combo)
		self.nodes_formGroupBox.setLayout(layout)

	def create_visualization_formGroupBox(self): 
		self.visualization_formGroupBox=QGroupBox("Inspection Visualization")
		layout=QFormLayout()
		layout.addRow(self.show_all_nodes, self.forcasting)
		layout.addRow(self.show_lanemarker, self.show_road_condition)
		layout.addRow(self.button_visualize)
		self.visualization_formGroupBox.setLayout(layout)
	
	def create_annotation_formGroupBox(self): 
		self.annotation_formGroupBox=QGroupBox("Inteactive Annotation")
		layout=QFormLayout()
		layout.addRow(self.button_inteactive_annotation)
		self.annotation_formGroupBox.setLayout(layout)

	def showdialog_information(self, title, message): 
		msg=QMessageBox()
		msg.setWindowTitle(title)
		msg.setIcon(QMessageBox.Information)
		msg.setText(message)
		msg.setStandardButtons(QMessageBox.Ok)
		retval=msg.exec_()
		return retval

	def showdialog_warnning(self, title, message): 
		msg=QMessageBox()
		msg.setWindowTitle(title)
		msg.setIcon(QMessageBox.Critical)
		msg.setText(message)
		msg.setStandardButtons(QMessageBox.Ok)
		retval=msg.exec_()
		return retval
	
	def showdialog_question(self, title, message): 
		reply=QMessageBox.question(self, title, message, 
		QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
		return reply

#%%############## Road inspection ######################
	def get_data_path(self): 
		# open select folder dialog
		if self.data_source.isChecked()==True: 
				data_path=QFileDialog.getOpenFileName(self, f'Select a mission file ', self.root_folder, "Images (*.png *.jpeg *.jpg *.bmp )")[0]
		else:
				data_path=QFileDialog.getExistingDirectory(self, 'Select the mission folder ', self.root_folder)

		self.root_folder=os.path.dirname(data_path)

		return data_path

	def get_image_path(self): 
			self.image_path=self.get_data_path()
			if self.image_path=='':
				return 0
			
			# process the slected data
			self.input_data_button.setText(self.image_path)
			if os.path.isdir(self.image_path):

				## Hide the insection and annotatin widgets
				self.setVisible_inspection_widgets(False)
				self.visualization_formGroupBox.setVisible(False)
				self.annotation_formGroupBox.setVisible(False)

				# uodate the new nodes/mission/sensors
				self.explore_nodes_and_missions(self.image_path)
			else:
				self.nodes_formGroupBox.setVisible(False)

	def define_nodes_and_missions(self, root):
		camera_folders=[]
		camera_folders=utils.getListOfFolders(root, ext='.jpg', path_pattern='sweeps')
		print(f'\n - The data folder contained {len(camera_folders)} mission. ')
		
		# find the list of nodes
		list_nodes=[]
		list_missions=[]
		for k, path in enumerate(camera_folders):
			# get the node and missions paths
			sweep_root=path.split('sweeps')[0]
			sweep_root=sweep_root.replace('\\', '/')
			if sweep_root[-1]=='/':
				sweep_root=sweep_root[:-1]
			# input(f'\n sweep_path={sweep_root}')

			node_=os.path.basename(sweep_root)
			list_nodes.append(node_)
			list_missions.append(sweep_root)

		# build the nodes dict
		node_dict={}
		
		for node_name in list_nodes:
			cnt=1
			missions_dict={}
			for sweep_root in list_missions:
				if node_name == os.path.basename(sweep_root):
					# updaet the mission list
					mission_name=f'mission{cnt}'
					mission_={mission_name:sweep_root}
					missions_dict.update(mission_)
					cnt+=1		
			# upadet teh node_dict
			node_dict.update( {node_name:missions_dict})
		self.node_dict=node_dict
		# print(f'\n\n node dict = \n{self.node_dict}')

	def explore_nodes_and_missions(self, root):

		self.define_nodes_and_missions(root)
		# update th combo:
		list_nodes=list(self.node_dict.keys())
		# print(f'\n flag: \n - list_nodes={list_nodes}')
		self.list_nodes_combo.clear()
		self.list_nodes_combo.addItem("select a node")
		self.list_nodes_combo.addItems(list_nodes)

		# set the current item
		self.list_nodes_combo.setCurrentIndex(1)
		self.on_node_selection()		
		self.list_missions_combo.setCurrentIndex(1)
		self.on_mission_selection()
		self.list_sensors_combo.setCurrentIndex(2)
		self.on_sensor_selection()
		# how/hide dropdowns
		self.nodes_formGroupBox.setVisible(True)

	def inspection_thread(self):
		# Step 2: Create a QThread object
		self.thread = QThread()
		# Step 3: Create a worker object
		self.worker = Worker()
		self.worker.N=len(self.list_files)
		# Step 4: Move worker to the thread
		self.worker.moveToThread(self.thread)
		# Step 5: Connect signals and slots
		self.thread.started.connect(self.worker.run)
		self.worker.finished.connect(self.thread.quit)
		self.worker.finished.connect(self.worker.deleteLater)
		self.thread.finished.connect(self.thread.deleteLater)
		self.worker.progress.connect(self.run_inspection_image_sequences)
		# Step 6: Start the thread
		self.thread.start()

	def	explore_database(self):
		'''
		Explore the created database
		'''
		from nuscenes.nuscenes import NuScenes
		nusc = NuScenes(version=self.version, dataroot=self.dataroot, verbose=True)

		# Show a scene 
		my_scene=nusc.scene[0]
		print(f'\n\n ==> List of recorded scenes: \n ')
		nusc.list_scenes()

		print(f'\n\n ==> First scene: \n {my_scene}')
		
		my_sample = nusc.get('sample', my_scene['first_sample_token'])
		print(f'\n\n ==> First sample of the scene: \n {my_sample}')

	def run_inspection_image_sequences(self, n):
		img_path=self.list_files[n]
		N=len(self.list_files)
		progress=f"[{n+1}/{N}]({int(100*n/N)}%)"
		self.inspection_image(img_path, progress)

		if n==N-1:
			self.setEnabled(True)
			self.button_run_inspection.setText('Run all frames inspection')
			self.button_run_inspection.setStyleSheet("background-color : QColor(53, 53, 53)")
		elif n==0:
			self.button_run_inspection.setText('Please wait till the process is done ...')
			self.button_run_inspection.setStyleSheet("background-color : red")
			self.setEnabled(True)


	def inspection_image(self, img_path, progress=''):
		self.image_ID.setText(f"{progress}\t fileID=" + os.path.basename(img_path))
		self.annotation_formGroupBox.setVisible(False)
		if '3D' in self.sensor_name or 'CSI' in self.sensor_name:
			# variation based inspection
			out_image, mask, nb_damages, dict_damage = \
					inspection_algorithm.inspection_diff(	img_path, bright_th=bright_th,
																							erosion_tol=erosion_tol, cnt_th=cnt_th, 
																							cnt_size_ratio=cnt_size_ratio, disp=False)
		
			# Display the diagnosed frame
			out_image = cv2.resize(out_image, resize) 
			n,m, _=out_image.shape
			if nb_damages>1:
					color=(0,0,0)
					cv2.putText(out_image, f'{nb_damages} damages: {str(dict_damage)}', 
					(int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
							0.6, color, 2)

		elif self.sensor_name=='RIGHT_CAMERA' or self.sensor_name=='LEFT_CAMERA':
			# DSP-based road segmentation
			reflection_coef, out_image, lane, lane_mrk_mask=inspection_algorithm.lane_inspection(img_path, disp=0)#, bright_th=bright_th)

			# Display the diagnosed frame
			out_image = cv2.resize(out_image, resize) 
			n,m, _=out_image.shape
			if reflection_coef<0.2:
					color=(0,0,255)
			else:
					color=(255, 255, 255)
			cv2.putText(out_image, f'reflection coef={reflection_coef:.2}', 
									(int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
										0.8, color, 2)
		else:
			msg=f"Undefined algorithm for the sensor: \n - {self.sensor_name}."
			QMessageBox.warning(self, "Inspection algorithm", msg )
			print(msg)
			return
	
		
		# visualize the image
		self.visualize_image(out_image)

		# # flag: explore the slected database
		# try:
		# 	self.explore_database()
		# except Exception as e:
		# 	print(f'\n\n - Error in loading the Nuscenes-like database.')
		# 	raise e

		# enable annotation 
		self.annotation_formGroupBox.setVisible(True)

	def napari_mask_verification(self): 
		global tool_name
		# run the inspection on Napari GUI
		self.mask_path=self.image_path
		ret_val, msg_out=Napari_GUI(self.image_path, self.mask_path,
									annotation_directory=self.annotation_directory)

class App(QWidget): 
	def __init__(self, config_file='../config/config.yml'): 
		super().__init__()
		# parameters
		self.disp=1
		# instanciate the HAIS	GUI
		self.HAIS=HAIS_GUI(config_file=config_file, disp=self.disp)

#%%#
def set_GUI_style(app): 
		# Force the style to be the same on all OSs: 
		app.setStyle("Fusion")
		# Now use a palette to switch to dark colors: 
		palette=QPalette()
		palette.setColor(QPalette.Window, QColor(53, 53, 53))
		palette.setColor(QPalette.WindowText, Qt.white)
		palette.setColor(QPalette.Base, QColor(25, 25, 25))
		palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
		palette.setColor(QPalette.ToolTipBase, Qt.black)
		palette.setColor(QPalette.ToolTipText, Qt.white)
		palette.setColor(QPalette.Text, Qt.white)
		palette.setColor(QPalette.Button, QColor(53, 53, 53))
		palette.setColor(QPalette.ButtonText, Qt.white)
		palette.setColor(QPalette.BrightText, Qt.red)
		palette.setColor(QPalette.Link, QColor(42, 130, 218))
		palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
		palette.setColor(QPalette.HighlightedText, Qt.black)
		app.setPalette(palette)

def get_n_parents_folders_from_path(path, TAG='', n=2): 
		'''
		get n parent folder path structure from the input path
		'''
		def get_parent(path): 
				return os.path.basename(os.path.dirname(path)) 

		if n==1 or path=='': 
				return	os.path.join(get_parent(path), TAG)
		else: 
				n=n-1
				# if TAG=='': 
				#		 TAG=os.path.join(get_parent(path), TAG)
				# else: 
				#		 TAG=get_parent(path)
				TAG=os.path.join(get_parent(path), TAG)

				return get_n_parents_folders_from_path(os.path.dirname(path), TAG=TAG, n=n)

def get_file_tag(path): 
		TAG=os.path.basename(os.path.dirname(path)) + '--' + os.path.basename(path)
		return TAG

def get_tool_scan_names_path(mask_path): 
	tool_name=get_folder_tag( get_n_parents_folders_from_path(os.path.dirname(mask_path), TAG='', n=2)) #os.path.basename(os.path.dirname(os.path.dirname(self.mask_path)))
	scan_name=os.path.basename(os.path.dirname(mask_path))
	return tool_name, scan_name

def get_folder_tag(path): 
		TAG=os.path.basename(os.path.dirname(os.path.dirname(path))) + '--' + os.path.basename(os.path.dirname(path))
		return TAG

def Napari_GUI(image_path, mask_path, annotation_directory='annotations', pyqt_gui=True): 
	# automate_annotation_folder
	tool_name, scan_name=get_tool_scan_names_path(image_path)
	annotation_directory=os.path.join(annotation_directory, 'dataset', scan_name)

	# load input data an masks if availlable
	img_ref	=load_image(image_path)
	img_input=load_image(mask_path)

	# run the napari GUI
	napari.gui_qt()
	viewer=napari.Viewer(title=f'HAIS: Interactive annotation verification')	# no prior setup needed
	viewer.dims.ndisplay=2
	# visulaize the reference volume
	# input(f'flag: img_ref shape={img_ref.shape}')
	name=get_file_tag(image_path)
	viewer.add_image(img_ref, name=name)
	
	# # hide the reference data
	# viewer.layers[name].visible=False

	# visulaize the input volume
	viewer.add_labels(img_input, name='damage mask: '+get_file_tag(mask_path), opacity=0.6)

	if not pyqt_gui:
		napari.run(force=True)
	return 0, ''
