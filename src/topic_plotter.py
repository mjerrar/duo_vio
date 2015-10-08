#!/usr/bin/python
from __future__ import print_function, division

import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui

__author__ = 'nicolas'

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print('Invalid use of topic_plotter. Provide a single argument that is a log file created with rostopic_logger')
        sys.exit(-1)

    log_data = [line.strip().split(',') for line in open(sys.argv[1], 'r').read().splitlines()]

    data = dict()

    for item in log_data:
        if item[1] in data:
            data[item[1]]['x'].append(int(item[0])/1e9)
            data[item[1]]['y'].append(float(item[2]))
        else:
            print('Found topic: {}'.format(item[1]))
            data[item[1]] = {'x': [int(item[0])/1e9], 'y': [float(item[2])]}

    win = pg.GraphicsWindow(title=sys.argv[1])
    pg.setConfigOptions(antialias=True)
    plot = win.addPlot()
    plot.addLegend()

    idx = 0
    for key, value in data.iteritems():
        plot.plot(x=value['x'], y=value['y'], pen=pg.intColor(idx, hues=len(data)), name=key, symbol='o', symbolSize=4, symbolPen=pg.intColor(idx, hues=len(data)), symbolBrush=pg.intColor(idx, hues=len(data)))
        idx += 1

    QtGui.QApplication.instance().exec_()
