#!/bin/bash

cp roscore.conf /etc/init/roscore.conf
cp roscore-start /usr/sbin/roscore-start

cp nero.conf /etc/init/nero.conf
cp nero-start /usr/sbin/nero-start
cp nero-stop /usr/sbin/nero-stop

cp kinect-start /usr/sbin/kinect-start
cp kinect.conf /etc/init/kinect.conf

cp ps3-start /usr/sbin/ps3-start
cp ps3.conf /etc/init/ps3.conf

chmod +x /usr/sbin/roscore-start
chmod +x /usr/sbin/nero-start
chmod +x /usr/sbin/nero-stop
chmod +x /usr/sbin/kinect-start
