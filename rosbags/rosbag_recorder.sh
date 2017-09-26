#! /usr/bin/env bash

source devel/setup.bash &&

rosbag record /gps/fix  /encoder/encorders   nav440/nav440   /rslidar_packets
