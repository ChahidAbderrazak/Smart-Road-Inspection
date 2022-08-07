import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import QtCore, QtGui

# os.chdir('./lib/ultimatelabeling' )
from ultimatelabeling.views import *
from ultimatelabeling.models import State, StateListener, KeyboardNotifier
from ultimatelabeling.styles import Theme


def load_json(filename):
	try:
		if os.path.exists(filename):
			import json
			f = open(filename,)
			data = json.load(f)
			f.close()
		else:
			data=[]
		return data
	except:
		msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!!  \
			       \n --> a new Jason file will be created!!    '
		print(msg)
		# raise ValueError(msg)
		return []

def save_json(json_string, filename):
	import json
	try:
		# Using a JSON string
		with open(filename, 'w') as outfile:
			json.dump(json_string, outfile,indent=2)
			return 0
	except:
		print(f'\n\n - error in saving {filename}')
		return 1


class MainWindow(QMainWindow):
    def __init__(self,DATA_DIR, OUTPUT_DIR):
        super().__init__()
        self.DATA_DIR=DATA_DIR
        self.OUTPUT_DIR=OUTPUT_DIR
        self.setWindowIcon(QtGui.QIcon('files/icon.png'))
        self.setWindowTitle("HAIS Annotator using UltimateLabeler toolbox")
        

        self.central_widget = CentralWidget(DATA_DIR=self.DATA_DIR, OUTPUT_DIR=self.OUTPUT_DIR)
        self.central_widget.setFocusPolicy(Qt.StrongFocus)
        self.setFocusProxy(self.central_widget)
        self.central_widget.setFocus(True)

        self.statusBar()

        mainMenu = self.menuBar()

        fileMenu = mainMenu.addMenu('&File')
        helpMenu = mainMenu.addMenu('&Help')

        close = QAction('Close window', self)
        close.setShortcut('Ctrl+W')
        close.triggered.connect(self.close)
        fileMenu.addAction(close)

        open_action = QAction('Open a dataset', self)
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.open_dataset_click)
        fileMenu.addAction(open_action)

        import_action = QAction('Import annotations', self)
        import_action.setShortcut('Ctrl+I')
        import_action.triggered.connect(self.central_widget.io.on_import_click)
        fileMenu.addAction(import_action)

        export = QAction('Export annotations', self)
        export.setShortcut('Ctrl+E')
        export.triggered.connect(self.central_widget.io.on_export_click)
        fileMenu.addAction(export)

        """save = QAction('Save', self)
        save.setShortcut('Ctrl+S')
        save.triggered.connect()
        fileMenu.addAction(save)"""

        help = QAction('Documentation', self)
        help.triggered.connect(self.open_url)
        helpMenu.addAction(help)

        self.setCentralWidget(self.central_widget)
        
        self.showMaximized()
        self.show()
        self.center()
        
    def open_dataset_click(self): #change: Added function by Abderrazak
        global DATA_DIR
        # QMessageBox.information(self.parent, "Opening a new dataset", "Please open the folder where the sensors data are saved")
        DATA_DIR = QFileDialog.getExistingDirectory(self, "Opening a new dataset", options=QFileDialog.Options())
        if DATA_DIR is None or DATA_DIR == "":
            return
        conf_dict = load_json('config/annotation_config.json')
        conf_dict['DATA_DIR']=DATA_DIR
        save_json(conf_dict, 'config/annotation_config.json')
        self.central_widget.state=State(DATA_DIR=DATA_DIR, OUTPUT_DIR=DATA_DIR)
        # self.central_widget.state.update_dataset(DATA_DIR)
        self.central_widget.state.load_state()
        self.central_widget.video_list_widget = VideoListWidget(self.central_widget.state)

    def open_url(self):
        url = QtCore.QUrl('https://github.com/alexandre01/UltimateLabeling')
        if not QtGui.QDesktopServices.openUrl(url):
            QtGui.QMessageBox.warning(self, 'Open Url', 'Could not open url')

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def closeEvent(self, event):
        print("exiting")
        self.central_widget.ssh_login.closeServers()
        self.central_widget.state.track_info.save_to_disk()
        self.central_widget.state.save_state()


class CentralWidget(QWidget, StateListener):
    def __init__(self,DATA_DIR, OUTPUT_DIR):
        super().__init__()
        self.DATA_DIR=DATA_DIR
        self.OUTPUT_DIR=OUTPUT_DIR
        self.state = State(DATA_DIR=self.DATA_DIR, OUTPUT_DIR=self.OUTPUT_DIR)
        self.state.load_state()
        self.state.add_listener(self)
        self.keyboard_notifier = KeyboardNotifier()

        self.video_list_widget = VideoListWidget(self.state)
        self.img_widget = ImageWidget(self.state)
        self.slider = VideoSlider(self.state, self.keyboard_notifier)
        self.player = PlayerWidget(self.state)
        # self.theme_picker = ThemePicker(self.state)
        self.options = Options(self.state)
        self.ssh_login = SSHLogin(self.state)

        self.detection_manager = DetectionManager(self.state, self.ssh_login)
        self.tracking_manager = TrackingManager(self.state)
        self.hungarian_button = HungarianManager(self.state)
        self.info_detection = InfoDetection(self.state)

        self.io = IO(self, self.state)

        self.keyPressEvent = self.keyboard_notifier.keyPressEvent
        self.keyReleaseEvent = self.keyboard_notifier.keyReleaseEvent
        self.keyboard_notifier.add_listeners(self.player, self.slider, self.img_widget, self.info_detection,
                                             self.tracking_manager)

        # Avoid keyboard not being triggered when focus on some widgets
        self.video_list_widget.setFocusPolicy(Qt.NoFocus)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.setFocusPolicy(Qt.StrongFocus)

        # Image widget thread signal, update function should always be called from main thread
        self.img_widget.signal.connect(self.img_widget.update)
        self.state.img_viewer = self.img_widget

        self.make_layout()
        # self.on_theme_change()

    def make_layout(self):
        main_layout = QHBoxLayout()

        navbar_box = QGroupBox("Videos")
        navbar_layout = QVBoxLayout()
        navbar_layout.addWidget(self.video_list_widget)
        navbar_box.setLayout(navbar_layout)
        main_layout.addWidget(navbar_box)

        image_box = QGroupBox("Image")
        image_layout = QVBoxLayout()
        image_layout.addWidget(self.img_widget)
        image_layout.addWidget(self.slider)
        image_box.setLayout(image_layout)
        main_layout.addWidget(image_box)

        control_box = QGroupBox("Control")
        control_layout = QVBoxLayout()
        control_layout.addWidget(self.player)
        control_layout.addWidget(self.ssh_login)
        # control_layout.addWidget(self.theme_picker)
        control_layout.addWidget(self.options)
        control_layout.addWidget(self.detection_manager)
        control_layout.addWidget(self.hungarian_button)
        control_layout.addWidget(self.tracking_manager)
        control_layout.addWidget(self.info_detection)

        control_layout.addStretch()
        control_box.setLayout(control_layout)
        main_layout.addWidget(control_box)

        self.setLayout(main_layout)

    def on_theme_change(self):
        app.setStyle("Fusion")
        app.setPalette(Theme.get_palette(self.state.theme))

def run_2D_annotator(DATA_DIR, OUTPUT_DIR):
    import os
    app = QApplication([])
    app.setStyle("Fusion")


    main_window = MainWindow(DATA_DIR=DATA_DIR, OUTPUT_DIR=OUTPUT_DIR)
    app.exec()

if __name__ == '__main__':
    # # Annotating using config file
    # conf_dict = load_json('config/annotation_config.json')
    # DATA_DIR = conf_dict['DATA_DIR']
    # OUTPUT_DIR = conf_dict['DATA_DIR']
    # run_2D_annotator(DATA_DIR=DATA_DIR, OUTPUT_DIR=OUTPUT_DIR)

    # Annotating using folder path
    conf_dict = load_json('config/annotation_config.json')
    DATA_DIR = '/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/dash-CAM/2022.08.05'
    OUTPUT_DIR = DATA_DIR 
    run_2D_annotator(DATA_DIR=DATA_DIR, OUTPUT_DIR=OUTPUT_DIR)


