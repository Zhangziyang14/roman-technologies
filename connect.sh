ifconfig wlan0 down
ifconfig wlan0 up
ifconfig -a wlan0 192.168.0.2 netmask 255.255.255.0
route add default gw 192.168.0.1 wlan0
iwconfig wlan0 essid Roman
