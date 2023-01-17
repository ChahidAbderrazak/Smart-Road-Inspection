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
from PyQt5 import QtCore, Qt
from PyQt5.QtWidgets import QApplication, QWidget, QGroupBox, QFormLayout, QLabel, QMessageBox, \
														QLineEdit, QFileDialog, QPushButton, QDialog, QCheckBox, QVBoxLayout,\
														QLabel, QSizePolicy, QScrollArea, QMessageBox, QComboBox, QGridLayout,  \
														QDialogButtonBox, QAction,QMenuBar, QRadioButton
# from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon, QFont, QPalette, QColor, QImage, QPixmap, QPalette
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
			self.stop_worker=False
			for i in range(self.N):
					time.sleep(0.2)
					self.progress.emit(i)
					# print(f'thread run{i}')
					if self.stop_worker:
						break
			self.finished.emit()

	def stop(self):
		self.stop_worker = True

class PreferencesForm(QDialog):
	def __init__(self, preferences_path):
		super(QDialog, self).__init__()
		self.preferences_path = preferences_path #preferences_path="bin/preferences.json"
		self.default_path = os.path.join(os.path.dirname(preferences_path), "preferences_default.json")
		self.bright_th = QLineEdit()
		self.erosion_tol = QLineEdit()
		self.cnt_th = QLineEdit()
		self.cnt_size_ratio = QLineEdit()
		self.load_perference()
		# create the preference forms
		self.create_form()
		
		buttonBox = QDialogButtonBox(QDialogButtonBox.Yes | QDialogButtonBox.No | QDialogButtonBox.Cancel)
		buttonBox.button(QDialogButtonBox.Yes).setText("Apply")
		buttonBox.button(QDialogButtonBox.No).setText("Cancel")
		buttonBox.button(QDialogButtonBox.Cancel).setText("Reset to Default")
		buttonBox.button(QDialogButtonBox.Yes).clicked.connect(self.save_apply_data)
		# buttonBox.button(QDialogButtonBox.No).clicked.connect(self.cancel_saving)
		# buttonBox.button(QDialogButtonBox.Cancel).clicked.connect(self.reset_values)
		
		mainLayout = QVBoxLayout()
		mainLayout.addWidget(self.formGroupBox)
		mainLayout.addWidget(buttonBox)
		self.setLayout(mainLayout)
		self.setWindowTitle("Preferences")
		self.setWindowFlags(QtCore.Qt.WindowMinimizeButtonHint | QtCore.Qt.WindowCloseButtonHint)

	def load_perference(self):
		self.setting = utils.load_json(self.preferences_path)
		self.bright_th.setText(str(self.setting['bright_th']))
		self.erosion_tol.setText(str(self.setting['erosion_tol']))
		self.cnt_th.setText(str(self.setting['cnt_th']))
		self.cnt_size_ratio.setText(str(self.setting['cnt_size_ratio']))

	def create_form(self):
		self.formGroupBox = QGroupBox('Preferences')
		layout = QFormLayout()
		layout.addRow(QLabel("bright_th"), self.bright_th)
		layout.addRow(QLabel("erosion_tol"), self.erosion_tol)
		layout.addRow(QLabel("cnt_th"), self.cnt_th)
		layout.addRow(QLabel("cnt_size_ratio"), self.cnt_size_ratio)
		self.formGroupBox.setLayout(layout)

	def save_apply_data(self):
		done = self.save_data()
		if done == True:
			self.close()
		
	def save_data(self):
		bright_th = int(self.bright_th.text())
		self.setting['bright_th']=bright_th
		# Update the setting preferences
		utils.save_json(self.setting, self.preferences_path)
		return True

class HAIS_GUI(QWidget): 
	def __init__(self, config_file, disp=0): 
			super(HAIS_GUI, self).__init__()
			self.preferences_path="bin/preferences.json"
			self.config_file=config_file
			self.config=self.load_config(config_file)
			self.root_folder=self.config['DATA']['data_root']
			self.version=self.config['DATA']['version']
			self.enable_nscene=self.config['DATA']['enable_nscene']
			self.verbose=self.config['DATA']['enable_nscene']
			self.DB=None
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
			self.createMenu()
			self.preferences_window = PreferencesForm(self.preferences_path)
			self.setting=self.preferences_window.setting
			self.refresh_preferences_values()
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
			self.frame_step=1
			self.button_previous_frame_inspection=QPushButton('inspect previous frame ') 
			self.button_previous_frame_inspection.setFont(QFont("Times", 10, QFont.Bold))
			self.button_previous_frame_inspection.clicked.connect(self.inspect_previous_frame)

			self.button_next_frame_inspection=QPushButton('inspect next frame ') 
			self.button_next_frame_inspection.setFont(QFont("Times", 10, QFont.Bold))
			self.button_next_frame_inspection.clicked.connect(self.inspect_next_frame)

			self.button_run_inspection=QPushButton('Run all frames inspection') 
			self.button_run_inspection.setFont(QFont("Times", 10, QFont.Bold))
			self.button_run_inspection.clicked.connect(self.inspection_thread)
			
			# visualize the inspectin and map 
			self.button_visualize=QPushButton('Show the inspection map') 
			self.button_visualize.setFont(QFont("Times", 10, QFont.Bold))
			self.button_visualize.clicked.connect(self.show_inspection_map)
			self.show_all_nodes=QCheckBox("all nodes")
			self.forcasting=QCheckBox("forcasting")
			self.show_lanemarker=QRadioButton("lanemarker")
			self.show_road_condition=QRadioButton("road conditon")

			# Annotation verification
			self.button_inteactive_annotation=QPushButton('Inteactive visualization') 
			self.button_inteactive_annotation.setFont(QFont("Times", 10, QFont.Bold))
			self.button_inteactive_annotation.clicked.connect(self.napari_mask_verification)

			# list of nodes
			self.list_nodes_combo = QComboBox(self)
			self.list_nodes_combo.clear()
			self.list_nodes_combo.addItem("select a node")
			self.list_nodes_combo.activated.connect(self.on_node_selection)

			# list of trips
			self.list_trips_combo = QComboBox(self)
			self.list_trips_combo.clear()
			self.list_trips_combo.addItem("select a trip")
			self.list_trips_combo.activated.connect(self.on_trip_selection)

			# list of trips
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
			self.create_execution_formGroupBox()
			self.setVisible_inspection_widgets(False)

			self.create_visualization_formGroupBox()
			self.visualization_formGroupBox.setVisible(False)

			

			# Compile the maun GUI layout
			mainLayout=QVBoxLayout()
			
			mainLayout.addWidget(QLabel())
			mainLayout.addWidget(self.database_formGroupBox)
			mainLayout.addWidget(self.nodes_formGroupBox)
			mainLayout.addWidget(self.image_ID)
			mainLayout.addWidget(self.scrollArea)
			mainLayout.addWidget(self.execution_formGroupBox)
			mainLayout.addWidget(self.visualization_formGroupBox)
			self.setLayout(mainLayout)
			self.setWindowTitle("Browse the collected data")
			self.setWindowTitle('HAIS-Inspection')
			self.setWindowIcon(QIcon('files/icon_hais.png'))
			self.setGeometry(1, 1, 580, 1200)
			# self.setWindowFlags(
			# 		Qt.WindowCloseButtonHint
			# )
			self.show()

	def createMenu(self):
		#/// menu: dataset
		self.myQMenuBar = QMenuBar(self)
		self.menu_stdrd = self.myQMenuBar .addMenu("&File")
		self.preferences_data = QAction("&Preferences")
		self.preferences_data.triggered.connect(self.set_preferences)
		self.menu_stdrd.addAction(self.preferences_data)

		self.close_windows_action = QAction("&Exit")
		self.close_windows_action.triggered.connect(self.close)
		self.menu_stdrd.addAction(self.close_windows_action)

		#/// menu: help
		self.help_menu = self.myQMenuBar.addMenu("&Help")
		self.about_action = QAction("About")
		self.help_menu.addAction(self.about_action)
		self.about_action.triggered.connect(self.show_about)

	def refresh_preferences_values(self):
		self.bright_th= self.setting['bright_th']
		self.erosion_tol= self.setting['erosion_tol']
		self.cnt_th= self.setting['cnt_th']
		self.cnt_size_ratio= self.setting['cnt_size_ratio']

	def set_preferences(self):	
		self.preferences_window.show()

	def show_about(self):
		text = "<center>" \
			"<h1> CT-Based Integrity Monitoring System (CTIMS) </h1>" \
			"&#8291;" \
			"<img src=files/logo.png>" \
			"</center>" \
			"<p>	This software processes CT scans of an object before and after its use for Vault maintenance. It uses new algorithms to detect defects and localize their positions/locations within this object<br/>" \
			"Email:amrb@nvscanada.ca<br/></p>"
		QMessageBox.about(self, "About CTIMS 2021", text)


#%%############## GUI Routines ######################

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
			
		self.trip_dict=self.node_dict[self.node_name]
		# print(f'\n - node_dict={self.node_dict }\n - node_name={self.node_name} \n - trips={self.trip_dict}')

		list_trips=list(self.trip_dict.keys())

		# print(f'\n flag: \n - list_trips={list_trips}')
		self.list_trips_combo.clear()
		self.list_trips_combo.addItem("select a trip")
		self.list_trips_combo.addItems(list_trips)

	def on_trip_selection(self):
		self.trip_name=str(self.list_trips_combo.currentText())
		if self.trip_name=='select a trip':
			return
		self.dataroot=self.trip_dict[self.trip_name]
		
		# get list of sensors
		if self.enable_nscene: # Create Nuscene like database
			self.DB=hais_database.HAIS_database(dataroot=self.dataroot, version=self.version, verbose=self.verbose)
			# get the list of file
			self.inspect_json_file=self.DB.inspect_json_file
			self.inspection_dict=self.DB.inspection_dict
			self.out_metric_arr=np.array([0 for k in range(len(self.inspection_dict['token']))])
			# print(f'\n  inspection_dict = {self.DB.inspection_dict}')

			# list sensors names
			list_sensors=self.DB.get_list_sensors()
			list_sensors=[k for k in list_sensors if 'CAM' in k or 'LIDAR' in k]
		else:
			# self.out_metric_arr=np.array([0 for k in range(len(self.list_files))])
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
		# retreive list of files
		self.list_files=[path for path in glob(os.path.join(self.sensor_folder, '*')) if os.path.isfile(path)]
		if self.enable_nscene:
			self.nb_data_samples=len(self.DB.inspection_dict['token'])
		else:
			self.nb_data_samples=len(self.list_files)

		if self.nb_data_samples>0:
			print(f'\n - {self.sensor_name} sensor has {len(self.list_files)} files.')
			# self.inspection_formGroupBox.setVisible(True)
			self.setVisible_inspection_widgets(True)
			self.visualization_formGroupBox.setVisible(True)
			# show the first file
			self.file_index=0
			self.image_path=self.list_files[self.file_index]
			self.visualize_image(self.image_path)
		else:
			msg=f'\n - No files are found in the database: \n - sensor={self.sensor_name} \n - folder ={self.sensor_folder}'
			QMessageBox.information(self, "Empty data folder Viewer", msg)

	def inspect_previous_frame(self):
		self.frame_step=-1
		# get the next image path
		self.update_image_path()
		# run the inspection
		self.inspect_image(self.image_path)

	def inspect_next_frame(self):
		self.frame_step=1
		# get the next image path
		self.update_image_path()
		# run the inspection
		self.inspect_image(self.image_path)

	def update_image_path(self):
		if self.enable_nscene:
			filename, self.car_position=self.DB.get_file_path_ego(self.sensor_name, n=self.frame_step)
			# print(f'\n flag: \n n={self.file_index}\n filename={filename}')
			if filename!='':
				self.image_path=os.path.join(self.dataroot, filename)
		else:
			# get the next image path
			self.file_index+=self.frame_step
			if self.file_index<0:
				self.file_index=len(self.list_files)-1
			elif self.file_index>=len(self.list_files):
				self.file_index=0
			self.image_path= self.list_files[self.file_index]

	def show_inspection_map(self):
		if self.show_all_nodes.isChecked(): # show all nodes
			list_trips=[]
			for node_name in self.node_dict.keys():
				trip_dict=self.node_dict[node_name]
				for trip_name in trip_dict.keys():
					list_trips.append(trip_dict[trip_name])
		else:# show only the current node
			list_trips=[self.trip_dict[trip_name] for trip_name in self.trip_dict.keys()]

		# save the visualized map
		
		inspection_map.visualize_map(list_trips, maps_root=self.maps_root, 
																show_lanemarker=self.show_lanemarker.isChecked())

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
		except Exception as e: 
			msg=f'\n\n Error: The config file [{config_file}] cannot be read correctly OR is not found!! \n Exception: {e}'
			print(msg)
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
		self.nodes_formGroupBox=QGroupBox("Nodes and trips")
		layout=QFormLayout()
		layout.addRow(QLabel("Nodes: "), self.list_nodes_combo)
		layout.addRow(QLabel("trips: "), self.list_trips_combo)
		layout.addRow(QLabel("Sensor: "), self.list_sensors_combo)
		self.nodes_formGroupBox.setLayout(layout)

	def setVisible_inspection_widgets(self, enable):
		self.scrollArea.setVisible(enable)
		# self.imageLabel.setVisible(enable)
		self.image_ID.setVisible(enable)
		self.execution_formGroupBox.setVisible(enable)

	def create_execution_formGroupBox(self): 
		self.execution_formGroupBox=QGroupBox("Inspection")
		self.save_annotations=QCheckBox("Save annotations")
		self.save_annotations.setChecked(False)
		layout=QGridLayout()
		layout.addWidget(self.save_annotations, 0,0)
		layout.addWidget(self.button_previous_frame_inspection, 1,0)
		layout.addWidget(self.button_next_frame_inspection, 1,1)
		layout.addWidget(self.button_run_inspection, 2, 0, 1, 2)
		self.execution_formGroupBox.setLayout(layout)

	def create_visualization_formGroupBox(self): 
		self.visualization_formGroupBox=QGroupBox("Visualization")
		layout=QGridLayout()
		layout.addWidget(self.show_all_nodes, 0,0)
		layout.addWidget(self.forcasting, 0,1)
		layout.addWidget(self.show_lanemarker, 0,2)
		layout.addWidget(self.show_road_condition, 0,3)
		layout.addWidget(self.button_visualize, 2, 0, 1, 4)
		layout.addWidget(self.button_inteactive_annotation, 3, 0, 1, 4)
		self.visualization_formGroupBox.setLayout(layout)
	
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
	def browse_data_path(self): 
		# open select folder dialog
		if self.data_source.isChecked()==True: 
				data_path=QFileDialog.getOpenFileName(self, f'Select a trip file ', self.root_folder, "Images (*.png *.jpeg *.jpg *.bmp )")[0]
		else:
				data_path=QFileDialog.getExistingDirectory(self, 'Select the trip folder ', self.root_folder)

		self.root_folder=os.path.dirname(data_path)

		return data_path

	def get_image_path(self): 
			self.image_path=self.browse_data_path()
			if self.image_path=='':
				return 0
			
			# process the slected data
			self.input_data_button.setText(self.image_path)
			if os.path.isdir(self.image_path):
				## Hide the insection and annotatin widgets
				self.setVisible_inspection_widgets(False)
				self.visualization_formGroupBox.setVisible(False)

				# uodate the new nodes/trip/sensors
				self.explore_nodes_and_trips(self.image_path)
			else:
				self.nodes_formGroupBox.setVisible(False)

	def define_nodes_and_trips(self, root):
		camera_folders=[]
		camera_folders=utils.getListOfFolders(root, ext='.jpg', path_pattern='sweeps')
		print(f'\n - The data folder contained {len(camera_folders)} trip. ')
		
		# find the list of nodes
		list_nodes=[]
		list_trips=[]
		for k, path in enumerate(camera_folders):
			# get the node and trips paths
			sweep_root=path.split('sweeps')[0]
			sweep_root=sweep_root.replace('\\', '/')
			if sweep_root[-1]=='/':
				sweep_root=sweep_root[:-1]
			# input(f'\n sweep_path={sweep_root}')

			node_=os.path.basename(sweep_root)
			list_nodes.append(node_)
			list_trips.append(sweep_root)

		# build the nodes dict
		node_dict={}
		
		for node_name in list_nodes:
			cnt=1
			trips_dict={}
			for sweep_root in list_trips:
				if node_name == os.path.basename(sweep_root):
					# updaet the trip list
					trip_name=f'trip{cnt}'
					trip_={trip_name:sweep_root}
					trips_dict.update(trip_)
					cnt+=1		
			# upadet teh node_dict
			node_dict.update( {node_name:trips_dict})
		self.node_dict=node_dict
		# print(f'\n\n node dict = \n{self.node_dict}')

	def explore_nodes_and_trips(self, root):

		self.define_nodes_and_trips(root)
		# update th combo:
		list_nodes=list(self.node_dict.keys())
		# print(f'\n flag: \n - list_nodes={list_nodes}')
		self.list_nodes_combo.clear()
		self.list_nodes_combo.addItem("select a node")
		self.list_nodes_combo.addItems(list_nodes)

		# set the current item
		self.list_nodes_combo.setCurrentIndex(1)
		self.on_node_selection()		
		self.list_trips_combo.setCurrentIndex(1)
		self.on_trip_selection()
		# find images/camera sensors
		idx_list=[i	for i in range(self.list_sensors_combo.count()) if 'CAM' in self.list_sensors_combo.itemText(i)]
		idx=2
		if len(idx_list)>0:
			idx=idx_list[0]

		self.list_sensors_combo.setCurrentIndex(idx)
		self.on_sensor_selection()
		# how/hide dropdowns
		self.nodes_formGroupBox.setVisible(True)

	def inspection_thread(self):
		# Step 2: Create a QThread object
		self.thread = QThread()
		# Step 3: Create a worker object
		self.worker = Worker()
		self.worker.N=self.nb_data_samples
		# Step 4: Move worker to the thread
		self.worker.moveToThread(self.thread)
		# Step 5: Connect signals and slots
		self.thread.started.connect(self.worker.run)
		self.worker.finished.connect(self.thread.quit)
		self.worker.finished.connect(self.worker.deleteLater)
		self.thread.finished.connect(self.thread.deleteLater)
		self.worker.progress.connect(self.run_inspect_image_sequences)
		# Step 6: Start the thread
		self.thread.start()

	def stop_thread(self):
			self.worker.stop()
			# self.thread.wait()

	def save_inspection_dict(self):
		# update the inspection dict
		print(f'\n - out_metric_arr shape={self.out_metric_arr.shape}')
		values_=list(self.out_metric_arr)
		values_metric=[int(k) for k in values_]
		self.DB.update_inspection_dict(self.module_name, values_metric)

	def run_inspect_image_sequences(self, n):
		N=self.nb_data_samples #len(self.list_files)
		progress=f"[{n+1}/{N}]({int(100*n/N)}%)"
		try:
			if self.enable_nscene:
				if n==0:
					self.frame_step=0
				else:
					self.frame_step=1
				# get the next image path
				self.update_image_path()
			else:
				self.image_path=self.list_files[n]
			# run the inspection on the frame
			out_metric=self.inspect_image(self.image_path, progress)
			#update the inspection values
			# print(f'\n report out_metric={out_metric}')
			self.out_metric_arr[n]=int(out_metric)
			
			if n==N-1:# n==50: #
				self.setEnabled(True)
				self.button_run_inspection.setText('Run all frames inspection')
				self.button_run_inspection.setStyleSheet("background-color : QColor(53, 53, 53)")
				# stop the thread
				self.stop_thread()

				# update the insepction report
				self.save_inspection_dict()
				return
			elif n==0:
				self.button_run_inspection.setText('Please wait till the process is done ...')
				self.button_run_inspection.setStyleSheet("background-color : red")
				self.setEnabled(True)
				
		except Exception as e:
			print(f'\n error in ruuning imags sequences!!\n Exception: {e}')
			pass

	def get_image_id(self, filepath):
		filepath=os.path.basename(filepath)
		file_id=os.path.splitext(filepath)[0]
		return file_id

	def inspect_image(self, img_path, progress=''):
		# try:
			# refresh the algorithm values
			self.refresh_preferences_values()
			image_id=self.get_image_id(img_path)
			msg=f"{progress}\t fileID=" + os.path.basename(img_path)
			self.image_ID.setText(msg)
			if 'DRONE' in self.sensor_name or '3D' in self.sensor_name or 'CSI' in self.sensor_name:
				self.module_name="metric"
				# variation based inspection
				out_image, image_RGB, mask, nb_damages, dict_damage = \
						inspection_algorithm.inspection_diff(	img_path, bright_th=self.bright_th,
																								erosion_tol=self.erosion_tol, cnt_th=self.cnt_th, 
																								cnt_size_ratio=self.cnt_size_ratio, disp=False)
				# get diagnosis
				out_metric, color= inspection_algorithm.get_road_diagnosis(nb_damages, dict_damage)
				# Display the diagnosed frame
				out_image = cv2.resize(out_image, resize) 
				n,m, _=out_image.shape
				if nb_damages>1:
						cv2.putText(out_image, f'{nb_damages} damages: {str(dict_damage)}', 
						(int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
								0.6, color, 2)
			elif self.sensor_name=='RIGHT_CAMERA' or self.sensor_name=='LEFT_CAMERA':
				self.module_name="Lanemarker"
				# run Lanemarker reflectivity
				reflection_coef, lanemarker_img, image_RGB, mask=inspection_algorithm.lane_inspection(img_path, disp=0)#, bright_th=bright_th)
				# get diagnosis
				out_metric, color= inspection_algorithm.get_lanemarker_diagnosis(reflection_coef)
				out_image = cv2.resize(lanemarker_img, resize) 
				# Display the diagnosed frame
				n,m, _=out_image.shape
				cv2.putText(out_image, f'reflection coef={reflection_coef:.2}', 
										(int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
											0.8, color, 2)
			else:
				msg=f"Undefined algorithm for the sensor: \n - {self.sensor_name}."
				QMessageBox.warning(self, "Inspection algorithm", msg )
				print(msg)
				return
		
			# export annotation:	the mask and image
			if self.save_annotations.isChecked():
				self.save_image_and_mask(self.node_name, self.trip_name, 
																self.sensor_name, image_id, image_RGB, mask)
			# visualize the image
			self.visualize_image(out_image)
			return  out_metric
	
		# except Exception as e:
		# 	print(f'\n\n - Error in running the inspection.')
		# 	# QMessageBox.warning(self, "Error in running the inspection", f'{e}')
		# 	# raise e

	def save_image_and_mask(self, node_name, trip_name, sensor_name, image_id, image, mask):
		if len(np.unique(mask))>1: # ignore empty or full masks
			mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
			mask[mask!=0]=255
			# print(f'\n flag: gray_mask shape={mask.shape}, segments={np.unique(mask)}')
			dst_folder=os.path.join(self.annotation_directory, node_name, f'{trip_name}__{sensor_name}')
			# save the image
			image_folder=os.path.join(dst_folder, 'images')
			utils.create_new_directory(image_folder)
			cv2.imwrite(os.path.join(image_folder, image_id + ".jpg"), image)

			# save the mask
			mask_folder=os.path.join(dst_folder, 'masks')
			utils.create_new_directory(mask_folder)
			cv2.imwrite(os.path.join(mask_folder, image_id + ".png"), mask)

			print(f'\n - the {sensor_name} annotations are saved in :\n{dst_folder}')
		else:
			print(f'\n - the {sensor_name} annotations are ignored \
							\n - mask segments={np.unique(mask)}')

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
	img_ref	=utils.load_image(image_path)
	img_input=utils.load_image(mask_path)

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
