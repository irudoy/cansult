#!/usr/bin/env python3
"""Send a single CAN frame via PCAN. Properly uninitializes before and after."""

import argparse
import sys
import can
from can.interfaces.pcan.basic import PCANBasic, PCAN_USBBUS1


def main():
    parser = argparse.ArgumentParser(description='Send a CAN frame')
    parser.add_argument('can_id', help='CAN ID, e.g. 0x66F')
    parser.add_argument('data', nargs='*', help='Data bytes in hex, e.g. 01 FF')
    parser.add_argument('--channel', default='PCAN_USBBUS1')
    parser.add_argument('--bitrate', type=int, default=500000)
    args = parser.parse_args()

    can_id = int(args.can_id, 0)
    data = bytes(int(b, 16) for b in args.data)

    pcan = PCANBasic()
    pcan.Uninitialize(PCAN_USBBUS1)

    bus = can.Bus(interface='pcan', channel=args.channel, bitrate=args.bitrate)
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
    finally:
        bus.shutdown()
        pcan.Uninitialize(PCAN_USBBUS1)


if __name__ == '__main__':
    main()
