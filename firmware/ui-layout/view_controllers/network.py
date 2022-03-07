from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController
from ui_elements.scrolling_menu import ScrollingMenu


class NetworkController(ViewWithOverlayController):
    def __init__(self, topbar, toggle_wifi_fn):
        super(NetworkController, self).__init__([topbar])
        self.topbar = topbar
        self.main_menu_index = 1

        self.menu = ScrollingMenu(3, entry_h=14, show_numbers=False)
        self.menu.topbar = topbar
        self.menu.add_entry("Connect", self.connect_fn)
        self.wifi_entry_index = self.menu.add_entry("Turn wifi off", self.toggle_wifi_fn)
        self.wifi_is_on = True

        self.send_toggle_wifi_msg = toggle_wifi_fn

        self.network_info = """NETGEAR94-5G
chansey.local
192.168.0.18

dodobot
10.0.4.1
pw: xxxxxx"""

    def toggle_wifi_fn(self):
        self.topbar.notify(0, "Setting wifi " + "off" if self.wifi_is_on else "on", 1000)
        self.send_toggle_wifi_msg(self.screen, self)

    def toggle_wifi(self):
        self.wifi_is_on = not self.wifi_is_on
        if self.wifi_is_on:
            self.menu.entries[self.wifi_entry_index].name = "Turn wifi off"
        else:
            self.menu.entries[self.wifi_entry_index].name = "Turn wifi on"
        self.menu.draw()
        self.draw_network_info()

    def connect_fn(self):
        self.screen.load_view(11)

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()
        self.menu.y = self.topbar.height + 5
        self.menu.menu_w = self.screen.width() // 2 + 1
        self.menu.add_screen(self.screen)
        self.menu.on_load()
        self.draw_network_info()

    def draw_network_info(self):
        text_x = self.menu.menu_w + self.menu.x
        text_y = self.menu.y
        row_size = 10
        prev_index = -1
        index = 0
        while index <= len(self.network_info):
            next_index = self.network_info.find("\n", index)
            if next_index == -1:
                next_index = len(self.network_info)
            text = self.network_info[index: next_index]
            self.screen.setCursor(text_x, text_y)
            self.screen.print(text)

            index = next_index + 1
            text_y += row_size

            if index == prev_index:
                print("Index not increasing! aborting")
                return
            prev_index = index

    def on_back(self):
        self.screen.load_view(self.main_menu_index)
        self.menu.on_unload()

    def draw_with_overlays(self):
        pass

    def on_up(self):
        self.menu.on_up()
        self.draw_network_info()

    def on_down(self):
        self.menu.on_down()
        self.draw_network_info()

    def on_enter(self):
        self.menu.on_enter()
