#!/usr/bin/env python

import sys

from minesweeper_rqt_stats.controller_stats import StatsPlugin
from rqt_gui.main import Main

plugin = 'controller_stats'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
