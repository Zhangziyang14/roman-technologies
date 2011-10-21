#!/bin/bash
#Script

service network-manager stop
ifconfig wlano down
ifconfig wlan0 up
ifconfig -a wlan0 192.168.0.1 netmask 255.255.255.0
route add default gw 192.168.0.1 wlan0
hostapd /etc/hostapd/hostapd.conf
pkill hostapd
service network-manager start
