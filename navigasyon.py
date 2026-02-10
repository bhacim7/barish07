import math


def signed_angle_difference(C, A):  # C as current_heading(magnetic_heading) and A as adviced_course

    # Açılar 0-360 arasında normalize edilir
    C %= 360
    A %= 360

    # Saat yönünde fark (pozitif)
    clockwise_diff = (A - C) % 360

    # Eğer fark 180'den küçükse ya da eşitse, pozitif döndür
    if clockwise_diff <= 180:
        return clockwise_diff
    else:
        # Açı farkı saat yönünün tersindeyse, negatif döndür
        return clockwise_diff - 360


def haversine(lat1, lon1, lat2, lon2):  # todo: add to git if success
    if None in (lat1, lon1, lat2, lon2):
        # Burada istersen 0 döndür veya bir önceki bearing değerini koru
        # ya da sadece None döndür
        return 2
    R = 6371000  # Dünya'nın yarıçapı (metre cinsinden)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    if None in (lat1, lon1, lat2, lon2):
        # Burada istersen 0 döndür veya bir önceki bearing değerini koru
        # ya da sadece None döndür
        return 0.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    bearing = math.atan2(x, y)
    bearing_degrees = (math.degrees(bearing) + 360) % 360  # 0–360 derece arasında

    return bearing_degrees


def destination_point(lat, lon, bearing_deg, distance_m):
    # lat/lon in degrees, bearing in degrees, distance in meters -> returns lat,lon degrees
    R = 6371000.0
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    b = math.radians(bearing_deg)
    d = distance_m
    lat2 = math.asin(math.sin(lat1) * math.cos(d / R) + math.cos(lat1) * math.sin(d / R) * math.cos(b))
    lon2 = lon1 + math.atan2(math.sin(b) * math.sin(d / R) * math.cos(lat1),
                             math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def generate_waypoints(lat1, lon1, lat2, lon2, spacing_m):
    total = haversine(lat1, lon1, lat2, lon2)
    bearing = calculate_bearing(lat1, lon1, lat2, lon2)
    if total <= 0.0001:
        return [(lat2, lon2)]
    n = max(1, int(math.floor(total / spacing_m)))
    wps = []
    for i in range(1, n + 1):
        d = min(i * spacing_m, total)
        wps.append(destination_point(lat1, lon1, bearing, d))
    # kesin hedefi de sona ekle (eğer yoksa)
    if len(wps) == 0 or (abs(wps[-1][0] - lat2) > 1e-9 or abs(wps[-1][1] - lon2) > 1e-9):
        wps.append((lat2, lon2))
    return wps


def shift_waypoint(lat, lon, bearing_to_wp, offset_m, side='starboard'):
    # side: 'starboard' -> bearing + 90, 'port' -> bearing - 90
    perp = bearing_to_wp + (90.0 if side == 'starboard' else -90.0)
    return destination_point(lat, lon, perp, offset_m)