from os import path
from time import sleep
from datetime import datetime

from pyrplidar import PyRPlidar

DEVICE_PATH = '/dev/rplidar'
BAUD_RATE = 256000
TIMEOUT = 3

if __name__ == '__main__':
    if not path.exists(DEVICE_PATH):
        print(f'No device found for: {DEVICE_PATH}')
        raise SystemExit

    print(f'Found RPLidar on path: {DEVICE_PATH}')
    print(f'Date and time: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

    lidar = PyRPlidar()
    lidar.connect(port=DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)

    try:
        info = lidar.get_info()
        for key, value in vars(info).items():
            print(f'{key.capitalize()}: {value}')

        health = lidar.get_health()
        print(f'Health: {health}')

    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        sleep(0.2)
        lidar.disconnect()