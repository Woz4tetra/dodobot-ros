import pygame
from gif import load_gif

from adafruit_gfx.color565 import *
from ui_elements.view_controller import ViewController


class SplashScreenController(ViewController):
    def __init__(self):
        super(SplashScreenController, self).__init__()
        self.gif = None

    def on_load(self):
        _, self.gif = load_gif("gifs/bw_bot_splash.gif")
        for frame, duration in self.gif:
            self.screen.fillScreen(ST77XX_BLACK)
            w = frame.get_width()
            h = frame.get_height()
            x = (self.screen.width() - w) // 2
            y = (self.screen.height() - h) // 2
            self.screen.screen.blit(frame, (x, y))
            # self.screen.draw()

            serial_name = "chansey"
            w, h = self.screen.getTextBounds(serial_name)
            self.screen.setCursor((self.screen.width() - w) // 2, self.screen.height() - h - 5)
            self.screen.print(serial_name)
            self.screen.draw()
            pygame.time.wait(duration)

        for brightness in range(255, -1, -5):
            self.screen.set_display_brightness(brightness)
        # pygame.time.wait(750)

        self.screen.load_view(1)

        for brightness in range(0, 255):
            self.screen.set_display_brightness(brightness)

    def draw(self):
        pass

