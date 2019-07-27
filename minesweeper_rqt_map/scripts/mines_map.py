#!/usr/bin/env python

import sys

from minesweeper_rqt_map.mines_map import MapPlugin
from rqt_gui.main import Main

plugin = 'mines_map'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
