#!/usr/bin/env python3
import sys
import serial
import time

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    s = serial.Serial(port, 115200, timeout=0.1)
    # Toggle DTR/RTS to reset device
    s.dtr = True
    s.rts = False
    time.sleep(0.05)
    s.dtr = False
    s.rts = True
    end = time.time() + 15
    try:
        while time.time() < end:
            line = s.readline()
            if line:
                try:
                    print(line.decode('utf-8', 'replace'), end='')
                except Exception:
                    print(line)
            else:
                time.sleep(0.05)
    finally:
        s.close()

if __name__ == '__main__':
    main()
