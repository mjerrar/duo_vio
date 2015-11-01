#!/usr/bin/python
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
    time.sleep(2)  # sleep 2 seconds to make sure all processes are there

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
