from os import path
from time import sleep
from rplidar import RPLidar, RPLidarException

DEVICE_PATH = '/dev/ttyUSB0'
BAUDS_TO_TRY = [115200, 256000, 460800, 1000000]

for baud in BAUDS_TO_TRY:
    print(f'\n--- Trying baud {baud} ---')
    lidar = None
    try:
        lidar = RPLidar(port=DEVICE_PATH, baudrate=baud, timeout=3)
        sleep(0.5)
        lidar.clean_input()
        sleep(0.2)
        info = lidar.get_info()
        print(f'SUCCESS at {baud}:')
        for key, value in info.items():
            print(f'  {key}: {value}')
        break
    except RPLidarException as e:
        print(f'Failed at {baud}: {e}')
    except Exception as e:
        print(f'Other error at {baud}: {e}')
    finally:
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
        sleep(1.0)