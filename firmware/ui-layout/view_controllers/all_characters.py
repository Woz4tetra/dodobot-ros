from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController


class AllCharactersController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(AllCharactersController, self).__init__([topbar])
        self.topbar = topbar
        self.main_menu_index = 1

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()
        self.screen.setCursor(0, self.topbar.height)
        self.screen.setTextWrap(True)
        for c in range(256):
            self.screen.print(chr(c))
        self.screen.setTextWrap(False)

    def on_back(self):
        self.screen.load_view(self.main_menu_index)
