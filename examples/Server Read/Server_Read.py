import bluetooth
import sys


DEVICE_NAME = 'ESP32test'
BT_ADDR = ''


if __name__ == '__main__':
    print('Hello, World!')


    # This is the MAC address of our bluetooth device
    # Ideally we would like a better means of finding the device we want,
    # but that wasn't our focus for this project
    #BT_ADDR = '78:21:84:80:08:56'
    PORT = 1

    # Device discovery
    print("Scanning for BT connections")
    devices = bluetooth.discover_devices(lookup_names=True)

    for item in devices:
        # assuming list is [mac_address, device_name]
        if item[1] == DEVICE_NAME:
            BT_ADDR = item[0]

    print(BT_ADDR)
    print(type(BT_ADDR))

    if BT_ADDR == '':
        print(f"Could not find BT device {DEVICE_NAME}")
        print("The program will now exit\n")
        sys.exit(1)
    print("BT device found!\n")

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((BT_ADDR, PORT))
    sys.exit(1)

    
    # Device connection
    print("Connecting to BT device")
    try:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((BT_ADDR, PORT))
    except Exception:
        print("Failed to connect to the device")
        print("Make sure the device is already paired")
        print("The program will now exit\n")
        sys.exit(1)
    print("BT device connected!\n")