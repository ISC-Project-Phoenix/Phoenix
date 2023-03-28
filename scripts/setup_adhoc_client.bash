#!/bin/bash

# Sets up the adhoc network used to connect the onboard PC to utilibot

# Find interface with mesh support
device="$(python3 find_adhoc_interface.py)"

ip link set down "$device"
iw dev "$device" set type ibss
ip link set up "$device"
iw dev "$device" ibss join timebay 5745 HT40+

ip a add 192.168.0.2/24 dev "$device"
