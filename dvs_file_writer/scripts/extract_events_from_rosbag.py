#!/usr/bin/python

import rosbag
import sys

# check for correct usage
if len(sys.argv) < 2:
    sys.exit("Usage: " + sys.argv[0] + " bag_name [topic_name]")

# load parameters
bag_name = sys.argv[1]

if (len(sys.argv) >= 3):
  topic_name = sys.argv[2]
else:
  topic_name = "/dvs/events"

print "Processing file: " + bag_name
    
i = 0
with rosbag.Bag(bag_name, 'r') as bag:
  target = open(bag_name + ".txt", 'w')
  
  for topic, msg, t in bag.read_messages():
    if topic == topic_name:
      for e in msg.events:
        i = i + 1
        target.write(str(e.x) + " " + str(e.y) + " " + str(1 if e.polarity else 0) + " " + str(e.ts.to_nsec()/1000) + "\n")
        
  target.close()
  print "Wrote " + str(i) + " events into text file " + bag_name + ".txt"
