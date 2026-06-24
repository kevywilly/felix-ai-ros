#!/usr/bin/env python3
"""Stop/start the RPLIDAR spin motor over serial -- NO ROS node required.

Talks directly to /dev/rplidar (a USB-serial device) using the Slamtec serial
protocol, so it works when nothing else is running. Motor control differs by
model, so we drive both:
  * DTR line          -- A1 (motor is HELD off while DTR is asserted)
  * SET_MOTOR_PWM 0   -- A2/A3/S1 (motor stops at PWM 0)

Why this is fiddly on the A1: the A1 motor has no firmware latch -- DTR is a
hardware enable line that must STAY asserted to keep the motor off. Normally
closing the port lowers DTR (the tty HUPCL "hang up on close" behavior), which
releases the motor and it spins straight back up. We clear HUPCL so the line
state survives the process exiting. If your adapter still won't hold it, use
--hold to keep the port open (Ctrl-C releases the motor).

IMPORTANT: only one process may hold /dev/rplidar. If the ROS `rplidar_node`
is running, this open() fails with "device busy" -- stop that node first, or
use the service path (`ros2 service call /stop_motor std_srvs/srv/Empty`).

Usage:
    ./lidar_motor.py                 # stop the motor (persist via HUPCL clear)
    ./lidar_motor.py --hold          # stop and keep holding until Ctrl-C
    ./lidar_motor.py --invert        # flip DTR polarity (if --stop spins it UP)
    ./lidar_motor.py --start         # spin it back up
    ./lidar_motor.py --port /dev/ttyUSB0 --baud 256000   # A2/S1
"""
import argparse
import sys
import termios
import time

import serial

SYNC = 0xA5
CMD_STOP = 0x25          # stop scanning (no response)
CMD_SET_PWM = 0xF0       # set motor PWM (payload: uint16 little-endian)
DEFAULT_PWM = 660        # Slamtec default running PWM (max 1023)


def _clear_hupcl(ser: serial.Serial) -> None:
    """Stop the tty from lowering DTR/RTS when the port closes, so the A1
    motor-off state persists after this process exits."""
    fd = ser.fileno()
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~termios.HUPCL          # cflag is index 2
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _send_pwm(ser: serial.Serial, pwm: int) -> None:
    """SET_MOTOR_PWM request: A5 F0 02 <lo> <hi> <checksum>."""
    pkt = [SYNC, CMD_SET_PWM, 0x02, pwm & 0xFF, (pwm >> 8) & 0xFF]
    chk = 0
    for b in pkt:
        chk ^= b
    pkt.append(chk)
    ser.write(bytes(pkt))


def stop_motor(ser: serial.Serial, dtr_stop: bool) -> None:
    ser.write(bytes([SYNC, CMD_STOP]))   # halt scanning first
    time.sleep(0.01)
    _send_pwm(ser, 0)                    # A2/A3/S1: PWM off
    time.sleep(0.005)
    ser.dtr = dtr_stop                   # A1: assert DTR to cut the motor


def start_motor(ser: serial.Serial, dtr_stop: bool) -> None:
    ser.dtr = not dtr_stop               # A1: release DTR
    time.sleep(0.005)
    _send_pwm(ser, DEFAULT_PWM)          # A2/A3/S1: spin up


def main() -> int:
    p = argparse.ArgumentParser(description="Stop/start the RPLIDAR motor over serial (no ROS).")
    p.add_argument("--start", action="store_true", help="start the motor instead of stopping it")
    p.add_argument("--hold", action="store_true",
                   help="keep the port open holding DTR until Ctrl-C (for adapters that "
                        "won't retain the line after close)")
    p.add_argument("--invert", action="store_true",
                   help="flip DTR polarity (use if plain --stop spins the motor UP)")
    p.add_argument("--port", default="/dev/rplidar", help="serial device (default /dev/rplidar)")
    p.add_argument("--baud", type=int, default=256000,
                   help="baud: 256000 for A2 M12/A3/S1 (default), 115200 for A1")
    args = p.parse_args()

    # On the A1, DTR=True stops the motor on the standard adapter; --invert flips it.
    dtr_stop = not args.invert

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1, dsrdtr=False)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}", file=sys.stderr)
        print("       Is the ROS rplidar_node holding the port? Stop it first.", file=sys.stderr)
        return 1

    try:
        if not args.hold:
            _clear_hupcl(ser)            # persist line state past close()
        if args.start:
            start_motor(ser, dtr_stop)
            print(f"{args.port}: motor START sent")
        else:
            stop_motor(ser, dtr_stop)
            print(f"{args.port}: motor STOP sent")
            if args.hold:
                print("holding DTR -- press Ctrl-C to release (motor will spin back up)")
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nreleasing.")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
