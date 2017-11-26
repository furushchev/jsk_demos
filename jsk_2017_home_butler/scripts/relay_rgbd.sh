#!/bin/bash

ARGS=""
for OPT in "$@"; do
  case "$OPT" in
    __*)  # remove ROS arguments
      shift
      ;;
    *)
      ARGS="$ARGS $1"
      shift
      ;;
  esac
done

roslaunch jsk_2017_home_butler relay_rgbd.xml $ARGS --screen 1>&2
