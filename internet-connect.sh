ifconfig wlan0 down
ifconfig eth0 down
ifconfig eth0 up
ifconfig -a eth0 192.168.1.2 netmask 255.255.255.0
route add default gw 192.168.1.1 eth0
