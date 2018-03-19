#!/bin/bash

read -r -p "Put Ethernet name: " name ip

sudo echo "
auto iface $name inet static
address 192.168.1.$ip
netmask 24
gateway 192.168.1.1
">> /etc/network/interfaces
