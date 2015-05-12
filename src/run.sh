#!/bin/bash

# For debug use:
# > gdb ../bin/main
# > break exit
# > run ../conf/topology.conf
# > bt

if [ "$1" == "" ]; then
	echo "Usage: ./run.sh topology.conf"
else
	../bin/main ../conf/$1
fi

