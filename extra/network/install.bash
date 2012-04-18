#!/bin/bash

cp hostapd.conf /etc/hostapd/hostapd.conf
cp pandanet-start /usr/sbin/pandanet-start
chmod +x /usr/sbin/pandanet-start
cp pandanet.conf /etc/init/pandanet.conf
