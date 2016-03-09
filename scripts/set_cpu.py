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
# File: set_cpu.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division

import subprocess
import time


def get_pid(node_name):
    process = subprocess.Popen('rosnode info /{}'.format(node_name).split(), stdout=subprocess.PIPE)
    output = process.communicate()[0].splitlines()

    for line in output:
        idx = line.find('Pid:')
        if not idx < 0:
            return int(line[idx+4:])
    return -1


if __name__ == "__main__":
    time.sleep(5)  # sleep to make sure all processes are there

    # get pids
    command = 'pgrep -f ros'
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    pids = process.communicate()[0].splitlines()

    command = 'sudo -A cset proc -m -p {} --threads -t ros'.format(','.join(pids))
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    print(process.communicate()[0])
    
    for pid in pids:
        print('Changing scheduler for {}'.format(pid))
        command = 'sudo -A chrt -pav -r 99 {}'.format(pid)
        print(command)
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        print(process.communicate()[0])

    # print the cpuset status after moving the nodes to their sets
    print(subprocess.Popen('sudo -A cset set -l'.split(), stdout=subprocess.PIPE).communicate()[0])
