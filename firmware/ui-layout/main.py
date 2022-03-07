import os
import random
import pygame
from datetime import datetime
from adafruit_gfx import Adafruit_ST77XX
from adafruit_gfx.color565 import *
from ui_elements.view_controller import ViewController

from view_controllers.main_menu import MainMenuController
from view_controllers.splash_screen import SplashScreenController
from view_controllers.topbar import TopbarController
from view_controllers import topbar

from view_controllers.robot_status import RobotStatusController
from view_controllers.systems import SystemsController
from view_controllers.drive_motor_system import DriveMotorSystemController
from view_controllers.linear_system import LinearSystemController
from view_controllers.gripper_system import GripperSystemController
from view_controllers.camera_system import CameraSystemController
from view_controllers.system_info import SystemInfoController
from view_controllers.network import NetworkController
from view_controllers.connect_screen import ConnectScreenController
from view_controllers.all_characters import AllCharactersController
from view_controllers.brick_renderer import BrickRenderController

from gif import load_gif

VOLTAGE_V = 11.0


def on_event(tft, event):
    global VOLTAGE_V

    if event.type != pygame.KEYDOWN:
        return
    topbar_inst = tft.controllers[1].topbar
    if event.key == pygame.K_q:
        if topbar_inst.reporting_status == topbar.REPORTING_DISABLED:
            topbar_inst.reporting_status = topbar.REPORTING_ENABLED
        else:
            topbar_inst.reporting_status = topbar.REPORTING_DISABLED

    elif event.key == pygame.K_e:
        if topbar_inst.connection_status == topbar.NO_USB_POWER:
            topbar_inst.connection_status = topbar.USB_POWER_DETECTED
        elif topbar_inst.connection_status == topbar.USB_POWER_DETECTED:
            topbar_inst.connection_status = topbar.USB_CONNECTION_STABLE
        else:
            topbar_inst.connection_status = topbar.NO_USB_POWER

    elif event.key == pygame.K_w:
        if topbar_inst.motor_status == topbar.MOTORS_INACTIVE:
            topbar_inst.motor_status = topbar.MOTORS_ACTIVE
        else:
            topbar_inst.motor_status = topbar.MOTORS_INACTIVE

    elif event.key == pygame.K_h:
        topbar_inst.notify(2, "Not homed", 3.0)

    elif event.key == pygame.K_g:
        topbar_inst.notify(2, "Position error", 3.0)

    elif event.key == pygame.K_f:
        topbar_inst.notify(2, "Low battery", 6.0)

    elif event.key == pygame.K_v:
        VOLTAGE_V -= 1.0
        if VOLTAGE_V < topbar.CRITICAL_VOLTAGE - 1.0:
            VOLTAGE_V = topbar.FULL_VOLTAGE

    elif event.key == pygame.K_r:
        tft.load_view(0)

    elif event.key == pygame.K_t:
        _, gif = load_gif("gifs/chansey.gif")
        for frame, duration in gif:
            tft.fillScreen(ST77XX_BLACK)
            w = frame.get_width()
            h = frame.get_height()
            x = (tft.width() - w) // 2
            y = (tft.height() - h) // 2
            tft.screen.blit(frame, (x, y))
            tft.draw()
            pygame.time.wait(duration)

    elif event.key == pygame.K_c:
        tft.load_view(12)

    elif event.key == pygame.K_b:
        tft.load_view(13)


def update_topbar_time(tft):
    topbar_inst = tft.controllers[1].topbar
    if topbar_inst.connection_status == topbar.USB_CONNECTION_STABLE:
        topbar_inst.set_datetime_text(datetime.now().strftime("%I:%M:%S%p"))


def update_topbar_battery(tft):
    global VOLTAGE_V
    topbar_inst = tft.controllers[1].topbar
    topbar_inst.voltage_V = random.random() + VOLTAGE_V
    topbar_inst.current_mA = random.randint(900, 1100)


def toggle_wifi(tft, controller):
    controller.toggle_wifi()


def main():
    topbar = TopbarController()
    splash_screen = SplashScreenController()
    main_menu = MainMenuController(topbar)
    robot_status = RobotStatusController(topbar)
    systems = SystemsController(topbar)
    drive_motor_system = DriveMotorSystemController(topbar)
    linear_system = LinearSystemController(topbar)
    gripper_system = GripperSystemController(topbar)
    camera_system = CameraSystemController(topbar)
    system_info = SystemInfoController(topbar)
    network = NetworkController(topbar, toggle_wifi)
    connect_screen = ConnectScreenController(topbar)
    all_characters_screen = AllCharactersController(topbar)
    bricks = BrickRenderController()

    os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (0, 0)

    tft = Adafruit_ST77XX(on_event)

    tft.add_controller(splash_screen)
    tft.add_controller(main_menu)
    tft.add_controller(network, index=2)
    tft.add_controller(robot_status, index=3)
    tft.add_controller(systems, index=4)
    tft.add_controller(robot_status, index=5)
    tft.add_controller(drive_motor_system, index=6)
    tft.add_controller(linear_system, index=7)
    tft.add_controller(gripper_system, index=8)
    tft.add_controller(camera_system, index=9)
    tft.add_controller(system_info, index=10)
    tft.add_controller(connect_screen, index=11)
    tft.add_controller(all_characters_screen, index=12)
    tft.add_controller(bricks, index=13)

    tft.init(160, 128)
    # tft.init(320, 256)
    tft.set_delay(300)
    tft.set_scale(3.0)

    tft.set_display_brightness(255)
    tft.fillScreen(ST77XX_BLACK)
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK)

    tft.setTextWrap(False)
    tft.setTextSize(1)
    tft.setRotation(3)

    tft.print("Hello!\n")
    print("Display ready")

    tft.fillScreen(ST77XX_BLACK)

    # tft.load_view(0)
    tft.load_view(1)

    while tft.running:
        if not tft.update():
            continue

        # tft.fillScreen(ST77XX_BLACK)
        update_topbar_time(tft)
        update_topbar_battery(tft)

    pygame.quit()


if __name__ == '__main__':
    main()
