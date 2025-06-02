#!/bin/bash

# set this variable to specific device
export HOME_MONITOR_SYSTEM_DEVICE="/dev/bus/usb/001/011"

docker-compose -f home_monitor_system.yml up -d