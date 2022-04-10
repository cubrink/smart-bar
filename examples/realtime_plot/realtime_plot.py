import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import bluetooth
import sys

DEVICE_NAME = "ESP32test"
BT_ADDR = ""
PORT = 1


if __name__ == "__main__":
    # Connect to bluetooth device
    devices = bluetooth.discover_devices(lookup_names=True)

    for mac_address, device_name in devices:
        if device_name == DEVICE_NAME:
            BT_ADDR = mac_address
    if BT_ADDR == "":
        print(f"Could not find BT device {DEVICE_NAME}")
        print("The program will now exit\n")
        sys.exit(1)

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((BT_ADDR, PORT))

    while True:
        data = sock.recv(1024)
        if data:
            print(data)
    # # Figure
    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # xs = []
    # ys = []
