from pyubx2 import UBXReader
import serial

with serial.Serial("COM6", 9600, timeout=1) as port:
    ubr = UBXReader(port)
    for _, parsed in ubr:
        if parsed.identity == "NAV-PVT":
            print(parsed.lat, parsed.lon)
            break
