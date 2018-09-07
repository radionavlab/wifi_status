#!/usr/bin/python
from subprocess import call
import string
import rospy
import os

import mg_msgs.msg

INT='wlp4s0'

KEYS = [
    'interface',
    'status',
    'link_quality',
    'signal_level',
    'noise_level',
    'discarded_nwid',
    'discarded_crypt',
    'discarded_frag',
    'discarded_retry',
    'discarded_misc',
    'missed_beacon'
]

FMTS = [
        lambda x: x[:-1],
    str,
    float,
    float,
    float,
    int,
    int,
    int,
    int,
    int,
    int
]

def get_wifi_status():

    with open('/proc/net/wireless') as f:

        # Discard header lines
        f.readline()
        f.readline()

        out = {}

        for line in f:
            # Parse line
            fields = line.split()
            
            fmt_fields = [field[0](field[1]) for field in zip(FMTS, fields)]
            interface = dict(zip(KEYS, fmt_fields))

            out[interface['interface']] = interface
        
        return out

def do_ping(addr):

    command = 'ping -n -c 1 -W 0.5 {}'.format(addr)
    stream = os.popen(command)

    stream.readline()
    line = stream.readline()

if __name__  == "__main__":

    do_ping('1.1.2.3')

    print('I am initialize')

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', mg_msgs.msg.WifiStatus, queue_size=10)
    rate = rospy.Rate(10) # Hz
    
    while not rospy.is_shutdown():

        data = get_wifi_status().values()[0]

        print(data)

        pub.publish(**data)
        rate.sleep()
        print('whoop de loop')

