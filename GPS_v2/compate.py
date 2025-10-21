import math

# Hàm tính khoảng cách Haversine giữa hai điểm (lat/lon) trên Trái Đất
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # bán kính Trái Đất (m)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c  # khoảng cách (m)

# --- Ví dụ: so sánh ---
# Tọa độ thực tế
real_lat = 20.98155
real_lon =  105.78720

# Tọa độ GPS định vị được
gps_lat = 20.9815633
gps_lon = 105.7870425

distance = haversine(real_lat, real_lon, gps_lat, gps_lon)
print(f"Khoảng cách thực tế ↔ định vị: {distance:.2f} mét")
