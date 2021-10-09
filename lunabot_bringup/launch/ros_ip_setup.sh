#!/bin/bash

TESTING=true
IS_COMPUTER=true
COMPUTER_IP=172.20.10.10
ROBOT_IP=172.20.10.9

# RUN `. ./ros_ip_setup.sh` in your terminal after setting the above variables, check by doing `export | grep ROS`

if [ "$TESTING" = true ];
then # we want ROS master on the computer during testing to use Rviz and diagnostic tools for testing
  export ROS_MASTER_URI="http://$COMPUTER_IP:11311"

  if [ "$IS_COMPUTER" = true ];
  then
    export ROS_IP="$COMPUTER_IP"
  else
    export ROS_IP="$ROBOT_IP"
  fi
else # we want ROS master on the robot during the actual competition to reduce bandwitdth usage
  export ROS_MASTER_URI="http://$ROBOT_IP:11311"

  if [ "$IS_COMPUTER" = false ];
  then
    export ROS_IP="$ROBOT_IP"
  else
    export ROS_IP="$COMPUTER_IP"
  fi
fi