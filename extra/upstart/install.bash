#!/bin/bash

cp nero.conf /etc/init/nero.conf
cp nero-start /usr/sbin/nero-start
cp nero-stop /usr/sbin/nero-stop

chmod +x /usr/sbin/nero-start
chmod +x /usr/sbin/nero-stop
