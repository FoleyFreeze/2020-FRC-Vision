#!/bin/bash
#
cd /home/pi/2020-FRC-Vision
echo "starting vision"
source /home/pi/.virtualenvs/cv/bin/activate
python3 vision.py
echo "ending vision"
sleep 10
exit

