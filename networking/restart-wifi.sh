sudo nmcli radio wifi off
sudo rfkill unblock wlan
sudo systemctl restart NetworkManager.service
sudo ifconfig $1 up
sudo ifconfig wlan0 up
sleep 1
sudo service dnsmasq restart
sudo service hostapd restart
sudo nmcli radio wifi on
