ifconfig -a eth0 192.168.1.1 netmask 255.255.255.0

route add default gw 0.0.0.0 eth0
echo 1 > /proc/sys/net/ipv4/ip_forward
iptables-restore < /etc/iptables.sav
