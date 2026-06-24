from time import sleep
from pyrplidar import PyRPlidar

DEVICE_PATH = '/dev/rplidar'
BAUD_RATE = 256000
TIMEOUT = 3
MOTOR_PWM = 660   # ~10 Hz scan rate on A1; range 0-1023

NUM_SCANS = 5   # how many full 360 scans to capture before stopping


def main():
    lidar = PyRPlidar()
    lidar.connect(port=DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)

    # Spin up motor and let it reach steady speed.
    lidar.set_motor_pwm(MOTOR_PWM)
    sleep(2.0)

    try:
        info = lidar.get_info()
        health = lidar.get_health()
        print(f'Connected: {info}')
        print(f'Health: {health}')
        print(f'Capturing {NUM_SCANS} scans...\n')

        # start_scan() returns a generator FUNCTION; call it to get the iterator.
        scan_generator = lidar.start_scan()

        # pyrplidar yields measurements one at a time, not grouped scans.
        # We group them ourselves using each measurement's start_flag.
        current_scan = []
        scans_emitted = 0

        for measurement in scan_generator():
            if measurement.start_flag and current_scan:
                # Finished a full rotation; emit and reset.
                scans_emitted += 1
                current_scan.sort(key=lambda r: r[0])
                print(f'--- Scan {scans_emitted}: {len(current_scan)} points ---')
                for angle, distance in current_scan:
                    print(f'  {angle:7.2f} deg   {distance:8.1f} mm')

                if scans_emitted >= NUM_SCANS:
                    break
                current_scan = []

            # Drop bad measurements (quality 0 = no valid return).
            if measurement.quality > 0 and measurement.distance > 0:
                current_scan.append((measurement.angle, measurement.distance))

    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        print('\nShutting down...')
        lidar.stop()
        lidar.set_motor_pwm(0)
        sleep(0.2)
        lidar.disconnect()


if __name__ == '__main__':
    main()