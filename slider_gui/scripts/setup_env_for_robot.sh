#!/bin/bash
# A script to setup ROS_MASTER_URI and ROS_IP for directly connecting to a specific robot

INTERFACE=wlan0

if [ $# -eq 1 ]
then
  export ROS_MASTER_URI="http://${1}1:11311"
  echo "export ROS_MASTER_URI=$ROS_MASTER_URI"
  export ROS_IP=`ifconfig $INTERFACE | grep 'inet addr:'| cut -d: -f2 | awk '{ print $1 }'`
  echo "export ROS_IP=$ROS_IP"
else
  echo "Usage: source `basename $0` ROBOT-NAME"
fi

