import time
import serial


def main():
    usb_address = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_6809670-if00"
    usb_device = serial.Serial(usb_address, baudrate=9600)

    uart_address = "/dev/ttyTHS1"
    uart_device = serial.Serial(
        uart_address,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    try:
        while True:
            if usb_device.in_waiting > 0:
                line = usb_device.readline()
                line = line.decode()
                print("--> USB:", line)
            if uart_device.in_waiting > 0:
                line = uart_device.readline()
                line = line.decode()
                print("--> UART:", line)

            time.sleep(0.01)
    finally:
        usb_device.close()
        uart_device.close()

main()
