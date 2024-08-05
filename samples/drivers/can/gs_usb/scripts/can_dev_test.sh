#!/bin/bash

# Use command below to dump can messages on yout host with timestamps.
# candump any -tdHDex

[ "$EUID" -ne 0 ] && echo "This script must be run as root" && exit 1;

cand_dev_name=$1
bitrate=125000
num_msgs=32
can_id=456
txqueuelen=100
numtxqueues=4
numrxqueues=4

ip link set $cand_dev_name down
[ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name numtxqueues $numtxqueues
[ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name numrxqueues $numrxqueues
[ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name txqueuelen $txqueuelen
[ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name type can bitrate $bitrate
[ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name type can loopback off
[ $? -ne 0 ] && exit $?;

echo "Enabling termination"
ip link set dev $cand_dev_name type can termination 120  # use 0 or 120
# [ $? -ne 0 ] && exit $?;

# echo "Blinking NIC LED's for 3 sec."
# ethtool -p $cand_dev_name 3
# [ $? -ne 0 ] && exit $?;

ip link set $cand_dev_name up
[ $? -ne 0 ] && exit $?;

# echo "done" && exit 0;
for ((i=0; i < $num_msgs; i++)); do
	msg="bade$(printf "%.2x" $i)"
	cmd="cansend $cand_dev_name $can_id#$msg"
	echo $cmd
	$cmd
	[ $? -ne 0 ] && exit $?;
	sleep 0.1;
done

exit 0;
