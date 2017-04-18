#!/usr/bin/env python

import rosbag
import dvs_msgs
import argparse

parser = argparse.ArgumentParser(description='Count number of events in a rosbag')
parser.add_argument('--topic', default='/dvs/events')
parser.add_argument('bag')

args = parser.parse_args()

n_events = 0

for topic, msg, t in rosbag.Bag(args.bag).read_messages():
    # This also replaces tf timestamps under the assumption 
    # that all transforms in the message share the same timestamp
    if topic == args.topic:
        n_events += len(msg.events)
        print "# events: ", n_events
        