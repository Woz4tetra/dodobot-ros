from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController


class GripperSystemController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(GripperSystemController, self).__init__([topbar])
        self.topbar = topbar
        self.systems_index = 4

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()

    def on_back(self):
        self.screen.load_view(self.systems_index)
