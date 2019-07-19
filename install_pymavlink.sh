#!/bin/sh

SCRIPT=$(readlink -f "$0")
BASEDIR=$(dirname "$SCRIPT")
MAVLINKDIR=$BASEDIR/

echo $BASEDIR
export MDEF=$MAVLINKDIR/message_definitions
cd $MAVLINKDIR/pymavlink

python3 setup.py bdist_wheel
cd $BASEDIR

python3 -m pip install pymavlink --no-index --find-links=$MAVLINKDIR/pymavlink/dist
