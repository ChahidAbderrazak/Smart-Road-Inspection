import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel

def window(config_msg):
   app = QApplication(sys.argv)
   widget = QWidget()

   # display the message
   textLabel = QLabel(widget)
   textLabel.setText(f"Hello World!: \n {config_msg}\n")
   textLabel.move(110,85)

   # visualize the windows
   widget.setGeometry(50,50,320,200)
   widget.setWindowTitle("Docker App Tutorial")
   widget.show()
   sys.exit(app.exec_())
   
def prepare_parser(): 
	from argparse import ArgumentParser
	parser=ArgumentParser( description='Code template' )
	parser.add_argument( 
				"--cfg", 
				default="../config/config.yml", 
				metavar="FILE", 
				help="path to config file", 
				type=str, 
		 )
	return parser

if __name__ == '__main__':
	parser=prepare_parser()
	args=parser.parse_args()
	
	# connecting to DB
	msg=" the database is running"
	try:
		from lib import sql_utils
		mycursor = sql_utils.db.cursor() 
	except Exception as e:
		msg=" the database is down!!!"
		print(f"Error: cannot communicate with the DB \n {e}")

	# render the GUI
	window(f"- file={args.cfg} \n- message:{msg}")