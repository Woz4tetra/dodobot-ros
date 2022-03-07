from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController

from ui_elements.scrolling_menu import ScrollingMenu


class SystemsController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(SystemsController, self).__init__([topbar])
        self.topbar = topbar
        self.main_menu_index = 1
        self.drive_system_index = 6
        self.linear_system_index = 7
        self.gripper_system_index = 8
        self.camera_system_index = 9
        self.system_info_index = 10

        self.menu = ScrollingMenu(5, entry_h=14, show_numbers=True)
        self.menu.topbar = topbar

        self.menu.add_entry("Drive motors", self.drive_motors_menu_fn)
        self.menu.add_entry("Linear stepper", self.linear_stepper_menu_fn)
        self.menu.add_entry("Gripper", self.gripper_menu_fn)
        self.menu.add_entry("Camera", self.camera_menu_fn)
        self.menu.add_entry("System Info", self.system_info_fn)
        self.menu.add_blank(6)
        self.menu.add_entry("Menu 6", self.generic_menu_fn)
        self.menu.add_entry("Menu 7", self.generic_menu_fn)
        self.menu.add_entry("Menu 8", self.generic_menu_fn)
        self.menu.add_entry("Menu 9", self.generic_menu_fn)
        self.menu.add_blank(6)
        self.menu.add_entry("Menu 10", self.generic_menu_fn)
        self.menu.add_entry("Menu 11", self.generic_menu_fn)
        self.menu.add_entry("Menu 12", self.generic_menu_fn)

    def generic_menu_fn(self):
        print("Generic")

    def drive_motors_menu_fn(self):
        print("Drive motors")
        self.screen.load_view(self.drive_system_index)

    def linear_stepper_menu_fn(self):
        print("Linear stepper")
        self.screen.load_view(self.linear_system_index)

    def gripper_menu_fn(self):
        print("Gripper")
        self.screen.load_view(self.gripper_system_index)

    def camera_menu_fn(self):
        print("Camera")
        self.screen.load_view(self.camera_system_index)

    def system_info_fn(self):
        print("System")
        self.screen.load_view(self.system_info_index)

    def on_load_with_overlays(self):
        self.menu.y = self.topbar.height + 5
        self.menu.menu_w = self.screen.width() - self.menu.x - 5
        self.menu.add_screen(self.screen)
        self.menu.on_load()

    def on_back(self):
        self.screen.load_view(self.main_menu_index)
        self.menu.on_unload()

    def draw_with_overlays(self):
        pass

    def on_up(self):
        self.menu.on_up()

    def on_down(self):
        self.menu.on_down()

    def on_enter(self):
        self.menu.on_enter()
