sudo nmcli con add type wifi ifname wlan0 con-name dodobot-host autoconnect yes ssid Hostspot
sudo nmcli con modify dodobot-host 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
sudo nmcli con modify dodobot-host wifi-sec.key-mgmt wpa-psk
sudo nmcli con modify dodobot-host wifi-sec.psk "s0mething"
sudo nmcli con up dodobot-host
sudo nmcli connection modify dodobot-host connection.autoconnect no

