#!/usr/bin/env bash
BASE=$1
LIDAR=$2
echo "export MBSBASE=$BASE" >> ~/.bashrc
echo "export MBSLIDAR=$LIDAR" >> ~/.bashrc
source ~/.bashrc
