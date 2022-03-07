from adafruit_gfx.color565 import *
from ui_elements.view_controller import ViewController


class BrickRenderController(ViewController):
    def __init__(self):
        super(BrickRenderController, self).__init__()
        self.main_menu_index = 1
        self.search_path = "bricks_path.txt"
        self.code_mapping = {
            'w': ST77XX_WHITE,
            '#': ST77XX_WHITE,
            'b': ST77XX_BLUE,
            'g': ST77XX_GREEN,
            'r': ST77XX_RED,
            'c': ST77XX_CYAN,
            'm': ST77XX_MAGENTA,
            'y': ST77XX_YELLOW,
            'k': ST77XX_GRAY,
            'a': ST77XX_ORANGE,
        }
        self.border = 1

        self.start_x = 0
        self.start_y = 20
        self.brick_width = 20
        self.brick_height = 10

    def load_path(self):
        with open(self.search_path) as file:
            contents = file.read()
            line = ""
            for line in contents.splitlines():
                if line.strip().startswith("#"):
                    continue
                break
            return line

    def load_level(self, path):
        self.screen.fillScreen(ST77XX_BLACK)
        with open(path) as file:
            contents = file.read()

        x = self.start_x
        y = self.start_y
        for c in contents:
            if c == '\n':
                x = self.start_x
                y += self.brick_height
            elif c in self.code_mapping:
                brick_color = self.code_to_color(c)
                self.create_brick(x, y, self.brick_width, self.brick_height, brick_color)
                x += self.brick_width
            else:
                x += self.brick_width

    def on_load(self):
        self.load_level(self.load_path())

    def on_enter(self):
        self.load_level(self.load_path())

    def on_back(self):
        self.screen.load_view(self.main_menu_index)

    def code_to_color(self, code):
        return self.code_mapping[code]

    def create_brick(self, x, y, w, h, color):
        self.screen.fillRect(x + self.border, y + self.border, w - (2 * self.border), h - (2 * self.border), color)
