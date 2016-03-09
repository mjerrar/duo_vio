#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: topic_plotter.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division

import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

__author__ = 'nicolas'


def close_all():
    app = QtGui.QApplication([])
    app.closeAllWindows()

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print('Invalid use of {}. Provide at least one argument that is a log file created with rostopic_logger'.format(sys.argv[0]))
        sys.exit(-1)

    max_y = 0
    min_y = 100

    pg.mkQApp()
    windows = {}

    for filename in sys.argv[1:]:
        print('Generating plot for file {}'.format(filename))

        log_data = [line.strip().split(',') for line in open(filename, 'r').read().splitlines()]

        data = dict()

        for item in log_data:
            if item[1] in data:
                data[item[1]]['x'].append(int(item[0])/1e9)
                data[item[1]]['y'].append(float(item[2]))
            else:
                print('Found topic: {}'.format(item[1]))
                data[item[1]] = {'x': [int(item[0])/1e9], 'y': [float(item[2])]}

            if float(item[2]) > max_y:
                max_y = float(item[2])
            if float(item[2]) < min_y:
                min_y = float(item[2])

        windows[filename] = pg.GraphicsWindow(title=filename)
        windows[filename].show()
        pg.setConfigOptions(antialias=True)
        plot = windows[filename].addPlot()
        plot.addLegend()

        sh = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), windows[filename], None, close_all)
        sh.setContext(QtCore.Qt.ApplicationShortcut)

        idx = 0
        for key, value in data.iteritems():
            plot.plot(x=value['x'], y=value['y'], pen=pg.intColor(idx, hues=len(data), alpha=100), name=key, symbol='o', symbolSize=4, symbolPen=pg.intColor(idx, hues=len(data)), symbolBrush=pg.intColor(idx, hues=len(data)))
            idx += 1

    for filename in sys.argv[1:]:
        plot = windows[filename].getItem(0, 0)
        plot.setRange(yRange=[min_y, max_y])

    QtGui.QApplication.instance().exec_()
