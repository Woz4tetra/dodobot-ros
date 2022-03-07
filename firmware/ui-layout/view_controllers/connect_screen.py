import random
from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController
from ui_elements.scrolling_menu import ScrollingMenu


class ConnectScreenController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(ConnectScreenController, self).__init__([topbar])
        self.topbar = topbar
        self.network_index = 2

        self.menu = ScrollingMenu(3, entry_h=12, show_numbers=True)
        self.menu.topbar = topbar
        self.menu.add_entry("Refresh list", self.refresh_list)

        self.list_of_networks = []

    def refresh_list(self):
        networks = [
            "NETGEAR94",
            "NETGEAR94-5G",
            "410",
            "Apt3111",
            "Lan Solo",
            "Home-413",
            "XFINITY",
            "xfinitywifi",
        ]

        while len(self.menu.entries) > 1:
            self.menu.remove_entry(1)
        for _ in range(4):
            self.list_of_networks.append((random.choice(networks), random.random() * -10.0))

        for name, signal_strength in self.list_of_networks:
            self.menu.add_entry("%s %0.2f" % (name, signal_strength), self.select_entry)

        self.menu.on_load()
        self.screen.draw()

    def select_entry(self):
        self.topbar.notify(0, "Connect wifi", 1000)
        print(self.menu.selected_index)

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()
        self.menu.y = self.topbar.height + 5
        self.menu.menu_w = self.screen.width()
        self.menu.add_screen(self.screen)
        self.menu.on_load()

    def on_back(self):
        self.screen.load_view(self.network_index)

    def on_up(self):
        self.menu.on_up()

    def on_down(self):
        self.menu.on_down()

    def on_enter(self):
        self.menu.on_enter()