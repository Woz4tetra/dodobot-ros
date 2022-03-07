import re
import time
# import textwrap
import subprocess


class NetworkProxy:
    def __init__(self):
        self.connection_cmd = "nmcli -p -m multiline -f common device show".split(" ")
        self.list_cmd = "nmcli -p -m multiline -f common device".split(" ")
        self.hostname_cmd = ["hostname"]
        self.wifi_state_command = "nmcli radio wifi".split(" ")
        self.wifi_on_command = "nmcli radio wifi on".split(" ")
        self.wifi_off_command = "nmcli radio wifi off".split(" ")
        self.list_wifi_command = "nmcli -m multiline dev wifi list".split(" ")
        self.rescan_wifi_command = "nmcli dev wifi rescan".split(" ")
        self.hotspot_off_command = "nmcli con down %s"
        self.hotspot_on_command = "nmcli con up %s"

        self.device_regex = r"GENERAL\.DEVICE:(.*)"  # interface name
        self.device_state_regex = r"GENERAL\.STATE:.*\((.*)\)"  # connected or not
        self.connection_regex = r"GENERAL\.CONNECTION:(.*)"  # wifi SSID
        self.ip_address = r"IP4\.ADDRESS.*:(.*)"  # ip address

        self.device_regex = r"DEVICE:.* (.*)"  # device name
        self.device_state_regex_list = r"STATE:.* (.*)"  # connection state

        self.list_regex_templates = ["IN-USE", "SSID", "MODE", "CHAN", "RATE", "SIGNAL", "BARS", "SECURITY"]
        self.list_regexs = []
        for index, regex in enumerate(self.list_regex_templates):
            self.list_regexs.append(r"%s:(.*)" % self.list_regex_templates[index])

        # self.network_info_wrapper = textwrap.TextWrapper(width=17, break_long_words=True, replace_whitespace=False)

    def list_devices(self):
        result = subprocess.run(self.list_cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()

        devices = []
        matches = re.finditer(self.device_regex, output, re.MULTILINE)
        for match in matches:
            device_name = match.group(1)
            devices.append(device_name)

        conn_states = []
        matches = re.finditer(self.device_state_regex_list, output, re.MULTILINE)
        for index, match in enumerate(matches):
            state = match.group(1)
            conn_states.append(state)

        assert len(conn_states) == len(devices), f"{len(conn_states)} != {len(devices)}"

        report = {}
        for index in range(len(devices)):
            report[devices[index]] = conn_states[index]
        return report
    
    def connection_report(self, interface_name):
        result = subprocess.run(self.connection_cmd + [str(interface_name)], stdout=subprocess.PIPE)
        output = result.stdout.decode()

        device_name = "--"
        device_state = "--"
        connection_state = "--"
        ip_address = "xx.xx.xx.xx"

        match = re.search(self.device_regex, output)
        if match:
            device_name = match.group(1).strip()
        else:
            print("Device name doesn't match interface name! %s != %s" % (device_name, interface_name))

        match = re.search(self.device_state_regex, output)
        if match:
            device_state = match.group(1).strip()

        match = re.search(self.connection_regex, output)
        if match:
            connection_state = match.group(1).strip()

        match = re.search(self.ip_address, output)
        if match:
            ip_address = match.group(1).strip()
        
        return device_name, device_state, connection_state, ip_address

    def get_interface_info(self, interface_name):
        device_name, device_state, connection_state, ip_address = self.connection_report(interface_name)
        report = f"{device_name} {device_state}\n{connection_state}\n{ip_address}"
        return report

    def get_hostname(self):
        result = subprocess.run(self.hostname_cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return output.strip()

    def generate_report(self, devices):
        report = self.get_hostname() + ".local\n"
        for device, state in devices.items():
            if state == "unmanaged":
                continue
            device_report = self.get_interface_info(device)
            # report += "\n".join(self.network_info_wrapper.wrap(device_report))
            report += device_report
            report += "\n\n"
        return report.strip()

    def get_report(self):
        devices = self.list_devices()
        is_connected = False
        for state in devices.values():
            if state != "unmanaged":
                is_connected = True
                break

        if is_connected:
            report = self.generate_report(devices)
        else:
            report = "Disconnected"
        return report, devices

    def get_radio_state(self):
        result = subprocess.run(self.wifi_state_command, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return "enabled" in output
    
    def get_hotspot_state(self, interface_name, hotspot_name):
        _, _, connection_state, _ = self.connection_report(interface_name)
        return hotspot_name in connection_state

    def set_radio_state(self, state):
        if state:
            cmd = self.wifi_on_command
        else:
            cmd = self.wifi_off_command
        result = subprocess.run(cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return output
    
    def set_hotspot_state(self, state, hotspot_name):
        # https://gist.github.com/narate/d3f001c97e1c981a59f94cd76f041140
        # https://unix.stackexchange.com/questions/400382/nmcli-disconnect-without-turning-wifi-card-off
        if state:
            cmd = self.hotspot_on_command % hotspot_name
        else:
            cmd = self.hotspot_off_command % hotspot_name
        cmd = cmd.split(" ")
        result = subprocess.run(cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return output

    def get_list_report(self, device, length):
        info = self.wait_for_rescan(device)

        report = []
        for index in range(min(length, len(info))):
            device_info = info[index]
            bars = self.bars_to_int(device_info[6])
            line = [device_info[0] + device_info[1], bars]
            report.append(line)
        return report

    def wait_for_rescan(self, device):
        if not self.get_radio_state():
            return []

        result = subprocess.run(self.rescan_wifi_command, stdout=subprocess.PIPE)
        output = result.stdout.decode()

        info = []
        while len(info) <= 1:
            info = self.list_wifi(device)
            time.sleep(0.25)

        info.sort(key=self.sort_network_info, reverse=True)
        return info

    def list_wifi(self, device):
        # 0: IN-USE
        # 1: SSID
        # 2: MODE
        # 3: CHAN
        # 4: RATE
        # 5: SIGNAL
        # 6: BARS
        # 7: SECURITY

        result = subprocess.run(self.list_wifi_command, stdout=subprocess.PIPE)
        output = result.stdout.decode()

        info_list = []
        for subindex, regex in enumerate(self.list_regexs):
            matches = re.finditer(regex, output, re.MULTILINE)

            entries = []
            for match in matches:
                entry = match.group(1).strip()
                entries.append(entry)
            while len(info_list) < len(entries):
                info_list.append(["" for _ in range(len(self.list_regexs))])

            for index, entry in enumerate(entries):
                info_list[index][subindex] = entry
        return info_list

    def bars_to_int(self, bars):
        # _▂▄▆█ = [95, 9602, 9604, 9606, 9608]
        # ▂▄▆█ = b1111
        # ▂▄▆_ = b1110
        # ▂▄__ = b1100
        # ▂___ = b1000
        # ____ = b0000
        value = 0
        offset = 3
        for char in bars:
            char = ord(char)
            if char == 95:
                bit_value = 0
            else:
                bit_value = char - 9602 + 2
                bit_value //= bit_value
            value |= (bit_value << offset)
            offset -= 1

            if offset < 0:
                break
        return value

    @staticmethod
    def sort_network_info(item):
        # priorize "IN-USE" otherwise use "SIGNAL"
        if item[0] == "*":
            return 100000
        return int(item[5])


if __name__ == '__main__':
    def test():
        from pprint import pprint
        proxy = NetworkProxy()
        # devices = proxy.list_devices()
        # print(proxy.generate_report(devices))
        # pprint(proxy.wait_for_rescan("wlan0"))

        print(bin(proxy.bars_to_int("▂▄▆█")))
        print(bin(proxy.bars_to_int("▂▄▆_")))
        print(bin(proxy.bars_to_int("▂▄__")))
        print(bin(proxy.bars_to_int("▂___")))
        print(bin(proxy.bars_to_int("____")))
        print(bin(proxy.bars_to_int("▂▄_█")))
        print(bin(proxy.bars_to_int("_▄_█")))

    test()
