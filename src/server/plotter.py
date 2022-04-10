import matplotlib.pyplot as plt
import matplotlib.animation as animation
import bluetooth
import sys
import numpy as np

DEVICE_NAME = "ESP32test"
BT_ADDR = ""
PORT = 1


def animate(i, ys, sock):
    data = sock.recv(1024)
    if data[-2:] == b"\r\n":
        data = data[:-2]
    if (b"\r\n" not in data) and len(data) != 0:
        data = np.array(data.split(b","), dtype=np.float32)
        ys.append(data[0])

        # Draw x and y lists
        # print(f"{ys=}")
        ax.clear()
        ax.plot(ys)

        # Format plot
        plt.subplots_adjust(bottom=0.30)
        plt.title("Smart-bar Live Data")
        # plt.ylabel('Temperature (deg C)')


if __name__ == "__main__":
    # Connect to bluetooth device
    devices = bluetooth.discover_devices(lookup_names=True)

    print("Searching for devices...")
    for mac_address, device_name in devices:
        if device_name == DEVICE_NAME:
            BT_ADDR = mac_address
    if BT_ADDR == "":
        print(f"Could not find BT device {DEVICE_NAME}")
        print("The program will now exit\n")
        sys.exit(1)
    print(f"Device ({DEVICE_NAME}) found! Creating socket")
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((BT_ADDR, PORT))
    print("Socket created!")

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    # ys = np.empty((0, 10))
    pressure_diff = []
    rep_counter = []
    acceleration = []
    ys = []

    ani = animation.FuncAnimation(fig, animate, fargs=(ys, sock), interval=20)
    plt.show()

    while True:
        data = sock.recv(1024)
        try:
            if data[-2:] == b"\r\n":
                data = data[:-2]
            if data:
                data = np.array(data.split(b","), dtype=np.float32)
                ys.append(data[0])
                print(ys)
        except ValueError:
            print("Error", data)

    # plt.show()
