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
# File: ping_plotter.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division

__author__ = 'nicolas'

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import subprocess
import sys
import numpy as np
import time

"""
Plot the ping of an given IP.
Requires PyQtGraph

Use:
    ./ping_plotter.py 192.168.1.1
or
    rosrun vio_ros ping_plotter.py 192.168.1.1
"""

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print('Invalid use of {}. Provide exactly one IP address'.format(sys.argv[0]))
        sys.exit(-1)

    ip = sys.argv[1]
    window = pg.GraphicsWindow(title='Ping response time of {}'.format(ip))
    window.show()
    pg.setConfigOptions(antialias=True)
    p1 = window.addPlot(labels={'left': 'Ping response time [ms]'})
    plot = p1.plot(pen=(255, 0, 0), symbol='o', symbolSize=4, symbolPen=(255, 0, 0), symbolBrush=(225, 0, 0))

    plot_size = 100
    plot_data = -10*np.ones(plot_size)

    plot_idx = 0

    command = 'ping {}'.format(ip)
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    print(process.stdout.readline())

    while process.poll() is None:
        out = process.stdout.readline()
        time_idx = out.find('time=')
        print(out[:-1])
        if time_idx < 0:
            ping = -10
        else:
            ping = out[time_idx+5:-3]
        plot_data[0:-1] = plot_data[1:]
        plot_data[-1] = float(ping)
        plot.setData(plot_data)
        p1.setYRange(0, max(plot_data)+1, padding=0)
        # plot.setPos(plot_idx, 0)
        QtGui.QApplication.instance().processEvents()

        plot_idx += 1

        # if plot_idx >= plot_data.shape[0]:
        #     tmp = plot_data
        #     plot_data = np.empty(plot_data.shape[0]*2)
        #     plot_data[:tmp.shape[0]] = tmp

        time.sleep(1)
        QtGui.QApplication.instance().processEvents()
