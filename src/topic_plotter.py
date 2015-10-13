#!/usr/bin/python
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
        print('Invalid use of topic_plotter. Provide at least one argument that is a log file created with rostopic_logger')
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
            plot.plot(x=value['x'], y=value['y'], pen=pg.intColor(idx, hues=len(data)), name=key, symbol='o', symbolSize=4, symbolPen=pg.intColor(idx, hues=len(data)), symbolBrush=pg.intColor(idx, hues=len(data)))
            idx += 1

    for filename in sys.argv[1:]:
        plot = windows[filename].getItem(0, 0)
        plot.setRange(yRange=[min_y, max_y])

    QtGui.QApplication.instance().exec_()
