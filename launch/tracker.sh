#!/bin/bash

# localhost is not a valid ROS_MASTER_URI for multiple machines
export ROS_MASTER_URI="http://i61p134:11311"

# workaround: A Bug causes failure to check for known_hosts
export ROSLAUNCH_SSH_UNKNOWN=1

roslaunch asr_flock_of_birds_tracking tracker.launch
