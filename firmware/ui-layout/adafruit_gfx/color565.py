class Color565:
    def __init__(self):
        self.value = 0

    @classmethod
    def from_int(cls, value):
        self = cls()
        self.value = value
        return self

    @classmethod
    def from_rgb(cls, rgb):
        r, g, b = rgb
        self = cls()
        self.value = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        return self

    def to_rgb(self):
        b = ((self.value & 0x1F) << 3)
        g = ((self.value >> 3) & 0xFC)
        r = ((self.value >> 8) & 0xF8)
        return r, g, b

    def __eq__(self, other):
        return self.value == other.value

    def __repr__(self):
        return str(self.value)


ST77XX_BLACK = Color565.from_int(0x0000)
ST77XX_WHITE = Color565.from_int(0xFFFF)
ST77XX_RED = Color565.from_int(0xF800)
ST77XX_GREEN = Color565.from_int(0x07E0)
ST77XX_BLUE = Color565.from_int(0x001F)
ST77XX_CYAN = Color565.from_int(0x07FF)
ST77XX_MAGENTA = Color565.from_int(0xF81F)
ST77XX_YELLOW = Color565.from_int(0xFFE0)
ST77XX_ORANGE = Color565.from_int(0xFC00)
ST77XX_GRAY = Color565.from_rgb((128, 128, 128))
ST77XX_LIGHT_PINK = Color565.from_rgb((255, 154, 159))
ST77XX_LIGHT_BLUE = Color565.from_rgb((201, 217, 248))
ST77XX_DARKER_BLUE = Color565.from_rgb((61, 120, 216))
ST77XX_DARKER_GREEN = Color565.from_rgb((2, 208, 2))
ST77XX_DARK_GREEN = Color565.from_rgb((0, 220, 0))

if __name__ == '__main__':
    def conversion_test():

        colors = [
            ST77XX_BLACK,
            ST77XX_WHITE,
            ST77XX_RED,
            ST77XX_GREEN,
            ST77XX_BLUE,
            ST77XX_CYAN,
            ST77XX_MAGENTA,
            ST77XX_YELLOW,
            ST77XX_ORANGE,
        ]

        for color in colors:
            assert color.from_rgb(color.to_rgb()) == color


    def truth_table_test():
        truth_mapping = {
            "BLACK"  : (0x0000, (0, 0, 0)),
            "WHITE"  : (0xFFFF, (255, 255, 255)),
            "RED"    : (0xF800, (255, 0, 0)),
            "GREEN"  : (0x07E0, (0, 255, 0)),
            "BLUE"   : (0x001F, (0, 0, 255)),
            "CYAN"   : (0x07FF, (0, 255, 255)),
            "MAGENTA": (0xF81F, (255, 0, 255)),
            "YELLOW" : (0xFFE0, (255, 255, 0)),
            "ORANGE" : (0xFC00, (255, 128, 0)),
        }

        for color_name, (color_565, color_rgb) in truth_mapping.items():
            generated = Color565.from_rgb(color_rgb)
            assert generated.value == color_565, "[%s] %s != %s" % (color_name, generated.value, color_565)


    def test_colors():
        truth_table_test()
        conversion_test()
        print(hex(ST77XX_LIGHT_BLUE.value))
        print(hex(ST77XX_DARKER_BLUE.value))
        print(hex(ST77XX_DARKER_GREEN.value))
        print(hex(ST77XX_DARK_GREEN.value))


    test_colors()
