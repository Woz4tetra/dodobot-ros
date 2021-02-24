sudo nmcli radio wifi off
sudo rfkill unblock wlan

sudo ifconfig wlan1 192.168.4.1/24 up
sudo ifconfig wlan0 up
sleep 1
sudo service dnsmasq restart
sudo service hostapd restart

sudo nmcli radio wifi on
