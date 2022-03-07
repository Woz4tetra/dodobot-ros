# Setup access point on Dodobot

```bash
sudo apt-get update
sudo apt-get install hostapd dnsmasq   --assume-yes

sudo systemctl stop dnsmasq
sudo systemctl stop hostapd
```

Copy the following files:
```bash
/etc/dhcpcd.conf
/etc/dnsmasq.conf
/etc/hostapd/hostapd.conf
/etc/default/hostapd
/etc/sysctl.conf
/etc/network/interfaces
/etc/rc.local
```

Set routing:

```bash
sudo iptables -t nat -A  POSTROUTING -o wlan0 -j MASQUERADE
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
sed '$i/sbin/iptables-restore < /etc/iptables.ipv4.nat' /etc/rc.local | sudo tee /etc/rc.local > /dev/null
```

Stop network manager from controlling hotspot (copy file into place). Helpful forum post: https://askubuntu.com/questions/472794/hostapd-error-nl80211-could-not-configure-driver-mode:

`/etc/NetworkManager/NetworkManager.conf`

Apply NetworkManager changes and start access point:

```bash
sudo nmcli radio wifi off
sudo rfkill unblock wlan
sudo systemctl restart NetworkManager.service

sudo ifconfig wlan1 up
sudo ifconfig wlan0 up

sleep 1
sudo service dnsmasq restart
sudo service hostapd restart

sudo nmcli radio wifi on
```
