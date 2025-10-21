import serial
import csv
from datetime import datetime
import folium
import threading
import time
from pyubx2 import UBXReader, UBXMessage
import os

SERIAL_PORT = "COM7"
BAUDRATE = 115200
CSV_FILE = os.path.join("e:/GPS/GPS_v2", "gps_log.csv")
MAP_FILE = os.path.join("e:/GPS/GPS_v2", "gps_map.html")
UPDATE_INTERVAL = 5

gps_points = []

# ------------------ GHI LOG -------------------
def log_gps(lat, lon, height=0, speed=0, numSV=0):
    with open(CSV_FILE, mode='a', newline='') as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(['timestamp', 'latitude', 'longitude', 'height_m', 'speed_m_s', 'numSV'])
        writer.writerow([datetime.now(), lat, lon, height, speed, numSV])
    gps_points.append((lat, lon))
    print(f"[LOG] {datetime.now()} | Lat: {lat:.7f} Lon: {lon:.7f} H: {height:.2f} Speed: {speed:.2f} SV: {numSV}")

# ------------------ CẬP NHẬT BẢN ĐỒ -------------------
def update_map():
    while True:
        if gps_points:
            map_center = gps_points[0]
            m = folium.Map(location=map_center, zoom_start=18)
            for lat, lon in gps_points:
                folium.CircleMarker([lat, lon], radius=4, color='red').add_to(m)
            folium.PolyLine(gps_points, color="blue", weight=2.5, opacity=0.8).add_to(m)
            m.save(MAP_FILE)
            print(f"[MAP] Map updated: {MAP_FILE}")
        time.sleep(UPDATE_INTERVAL)

# ------------------ CẤU HÌNH GNSS -------------------
def configure_gnss(port):
    # 1️.Đặt tần số cập nhật 10Hz (CFG-RATE)
    msg_rate = UBXMessage("CFG", "CFG-RATE", POLL=False,
                          measRate=100,  # ms per measurement (10Hz)
                          navRate=1,
                          timeRef=0)
    port.write(msg_rate.serialize())
    time.sleep(0.5)
    print("[CFG] Set rate = 10 Hz")

    # 2️.Bật GPS + Galileo + BeiDou (CFG-GNSS)
    msg_gnss = UBXMessage("CFG", "CFG-GNSS", POLL=False,
        msgVer=0, numConfigBlocks=3,
        gnssConfig=[
            {'gnssId': 0, 'resTrkCh': 8, 'maxTrkCh': 16, 'enabled': 1},  # GPS
            {'gnssId': 2, 'resTrkCh': 4, 'maxTrkCh': 8,  'enabled': 1},  # Galileo
            {'gnssId': 3, 'resTrkCh': 4, 'maxTrkCh': 8,  'enabled': 1},  # BeiDou
        ])
    port.write(msg_gnss.serialize())
    print("[CFG] Enabled GPS, Galileo, BeiDou")

    # 3️.Lưu cấu hình (CFG-CFG)
    msg_save = UBXMessage("CFG", "CFG-CFG", POLL=False,
                          clearMask=0, saveMask=0xFFFF, loadMask=0)
    port.write(msg_save.serialize())
    print("[CFG] Configuration saved")

    time.sleep(1)

# ------------------ ĐỌC GPS -------------------
def read_gps():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as port:
        configure_gnss(port)  # Cấu hình GNSS trước khi đọc
        ubr = UBXReader(port)
        print("[INFO] Start reading NAV-PVT data...")

        for _, parsed in ubr:
            if parsed.identity == "NAV-PVT" and parsed.fixType >= 3 and parsed.gnssFixOk == 1:
                lat = parsed.lat
                lon = parsed.lon
                height = parsed.height / 1000
                speed = parsed.gSpeed / 1000
                numSV = parsed.numSV
                log_gps(lat, lon, height, speed, numSV)

# ------------------ MAIN -------------------
if __name__ == "__main__":
    threading.Thread(target=update_map, daemon=True).start()
    read_gps()
