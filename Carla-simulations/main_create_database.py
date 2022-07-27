
import numpy as np
from lib import utils
from lib import hais_manager


def main():
	# collected data input
	raw_data = hais_manager.HAIS_bot(config='config/config.json')

	# # create the database structure
	# raw_data.create_database()

	# annotate the a scene
	scene_=raw_data.scene
	print(f'\n\n - scene= {scene_}')
	raw_data.annotate_database(scene=scene_[0])


if __name__ == '__main__':
    main()
