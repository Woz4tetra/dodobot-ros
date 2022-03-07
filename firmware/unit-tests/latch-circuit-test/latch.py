import time
import serial
import subprocess


class ShutdownException(Exception):  pass


def shutdown():
    print("Shutdown function called. Shutting down everything.")
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()


def main():
    address = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_6809670-if00"
    device = serial.Serial(address, baudrate=9600)

    shutdown_timer = time.time()
    shutdown_starting = False
    shutdown_time_limit = 3.0  # seconds
    prev_display_countdown = None

    try:
        while True:
            if device.in_waiting > 0:
                line = device.readline()
                line = line.decode()
                line = line.splitlines()
                if line[-1] == "1":  # if button pushed,
                    shutdown_timer = time.time()
                    shutdown_starting = True
                elif line[-1] == "0":  # if button released,
                    shutdown_starting = False
                    prev_display_countdown = None
                    print("Canceled")

            if shutdown_starting:
                current_time = time.time()
                countdown_time = shutdown_time_limit - (current_time - shutdown_timer)
                countdown_time_int = int(countdown_time) + 1
                if countdown_time_int != prev_display_countdown:
                    print("%s..." % countdown_time_int)
                    prev_display_countdown = countdown_time_int
                if countdown_time <= 0.0:
                    print("Shutting down")
                    device.write(b"shutdown\n")
                    time.sleep(0.15)
                    raise ShutdownException

            time.sleep(0.01)
    except ShutdownException:
        device.close()
        shutdown()
    finally:
        device.close()

main()
