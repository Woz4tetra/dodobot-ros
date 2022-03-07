from .view_controller import ViewController


class ViewWithOverlayController(ViewController):
    def __init__(self, overlays):
        super(ViewWithOverlayController, self).__init__()

        self.overlays = overlays

    def _add_screen(self, screen):
        for overlay in self.overlays:
            overlay.screen = screen
        self.screen = screen

    def draw(self):
        for overlay in self.overlays:
            overlay.draw()
        self.draw_with_overlays()

    def on_load(self):
        for overlay in self.overlays:
            overlay.on_load()
        self.on_load_with_overlays()

    def on_unload(self):
        for overlay in self.overlays:
            overlay.on_unload()
        self.on_unload_with_overlays()

    def draw_with_overlays(self):
        pass

    def on_load_with_overlays(self):
        pass

    def on_unload_with_overlays(self):
        pass
