
import numpy as np
from lib import utils
from lib import hais_manager


def main():
	# collected data input
	raw_data = hais_manager.HAIS_bot(config='config.json')
	# create the database structure
	raw_data.create_database()

if __name__ == '__main__':
    main()
