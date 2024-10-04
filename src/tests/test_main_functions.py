

import os
import random
from argparse import ArgumentParser
from glob import glob

import numpy as np
from lib import utils


def prepare_parser():
    parser = ArgumentParser(description='Tester')
    parser.add_argument(
        "--cfg",
        default="config/config.yml",
        metavar="FILE",
        help="path to config file",
        type=str,
    )
    return parser


#####################################
parser = prepare_parser()
args = parser.parse_args()
config_file = args.cfg

# default inputs
config = utils.load_config(config_file)

def test_components_continuity_mask():
    img_path = 'data/road.jpg'
    return 0

