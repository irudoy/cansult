#!/usr/bin/env python3
"""CAN monitor for cansult — decodes diagnostic, data, and debug stream frames."""

import argparse
import atexit
import signal
import sys
import time
import can

STATES = ['STARTUP', 'INIT', 'POST_INIT', 'WAITING', 'IDLE', 'STREAMING']

CANSULT_IDS = {0x665, 0x666, 0x667, 0x668, 0x669, 0x66A, 0x66F}


def decode_0x665(d, prev):
    """Decode diagnostic frame, show deltas from previous."""
    state = STATES[d[0]] if d[0] < len(STATES) else str(d[0])
    delta = ''
    if prev:
        diffs = []
        labels = ['ORE', 'FE+NE', 'CAN', 'DMA', 'WDT']
        for i, label in enumerate(labels, 1):
            diff = (d[i] - prev[i]) & 0xFF
            if diff:
                diffs.append(f'+{diff} {label}')
        if diffs:
            delta = '  ' + ', '.join(diffs)
    return (f'DIAG  {state:10s} ORE={d[1]} FE+NE={d[2]} CAN={d[3]} '
            f'DMA={d[4]} WDT={d[5]} frames={d[6]} ms={d[7]*4}{delta}')


def decode_0x666(d):
    """Decode data frame 1: battery, coolant, timing, O2, TPS, AAC, AF."""
    return (f'DATA1 bat={d[0]*0.08:.1f}V cool={d[1]-50}C ign={110-d[2]}deg '
            f'O2={d[3]*10}mV TPS={d[4]*20}mV AAC={d[5]/2:.0f}% '
            f'AF={d[6]}% AF_sl={d[7]}%')


def decode_0x667(d):
    """Decode data frame 2: speed, RPM, injector, MAF."""
    rpm = (d[1] * 256 + d[2]) * 12.5
    inj = (d[3] * 256 + d[4]) / 100.0
    maf = (d[5] * 256 + d[6]) * 5
    return (f'DATA2 spd={d[0]*2}km/h rpm={rpm:.0f} '
            f'inj={inj:.2f}ms MAF={maf}mV')


def decode_0x668(d):
    """Decode data frame 3: bits, voltage, DTC, heartbeat."""
    return (f'DATA3 bit1=0x{d[0]:02X} bit2=0x{d[1]:02X} '
            f'Vdev={d[2]*0.08:.1f}V DTC=0x{d[6]:02X} hb={d[7]}')


def decode_debug(can_id, d, dlc):
    """Decode debug stream frames (raw UART hex)."""
    direction = 'RX' if can_id == 0x669 else 'TX'
    hexdata = ' '.join(f'{b:02X}' for b in d[:dlc])
    return f'DBG-{direction} [{dlc}] {hexdata}'


def format_msg(msg, prev_diag):
    """Format a CAN message, return (text, updated_prev_diag)."""
    d = msg.data
    aid = msg.arbitration_id

    if aid == 0x665:
        text = decode_0x665(d, prev_diag)
        return text, list(d)
    elif aid == 0x666:
        return decode_0x666(d), prev_diag
    elif aid == 0x667:
        return decode_0x667(d), prev_diag
    elif aid == 0x668:
        return decode_0x668(d), prev_diag
    elif aid in (0x669, 0x66A):
        return decode_debug(aid, d, msg.dlc), prev_diag
    else:
        hexdata = ' '.join(f'{b:02X}' for b in d[:msg.dlc])
        return f'0x{aid:03X} [{msg.dlc}] {hexdata}', prev_diag


def main():
    parser = argparse.ArgumentParser(description='CAN monitor for cansult')
    parser.add_argument('--channel', default='PCAN_USBBUS1')
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--ids', help='Filter CAN IDs, e.g. 0x665,0x669,0x66A')
    parser.add_argument('--output', help='Log to file instead of stdout')
    parser.add_argument('--stdout', action='store_true', help='Force stdout')
    args = parser.parse_args()

    id_filter = None
    if args.ids:
        id_filter = set(int(x, 0) for x in args.ids.split(','))

    # Always uninitialize PCAN channel first — previous process may not have cleaned up
    from can.interfaces.pcan.basic import PCANBasic, PCAN_USBBUS1
    pcan = PCANBasic()
    pcan.Uninitialize(PCAN_USBBUS1)

    bus = can.Bus(interface='pcan', channel=args.channel, bitrate=args.bitrate)

    def cleanup():
        bus.shutdown()

    atexit.register(cleanup)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    out = sys.stdout
    if args.output and not args.stdout:
        out = open(args.output, 'a', buffering=1)
        print(f'Logging to {args.output}', file=sys.stderr)

    prev_diag = None
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            if msg.arbitration_id not in CANSULT_IDS:
                continue
            if id_filter and msg.arbitration_id not in id_filter:
                continue

            text, prev_diag = format_msg(msg, prev_diag)
            ts = time.strftime('%H:%M:%S')
            print(f'{ts} {text}', file=out, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if out is not sys.stdout:
            out.close()


if __name__ == '__main__':
    main()
