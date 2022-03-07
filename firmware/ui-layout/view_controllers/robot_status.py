from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController


class RobotStatusController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(RobotStatusController, self).__init__([topbar])
        self.topbar = topbar
        self.main_menu_index = 1

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()

    def on_back(self):
        self.screen.load_view(self.main_menu_index)
