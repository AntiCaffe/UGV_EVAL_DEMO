import math

try:
    import pymap3d as _pymap3d
except ImportError:  # pragma: no cover - depends on field machine setup
    _pymap3d = None


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def enu_backend_name():
    return "pymap3d" if _pymap3d is not None else "wgs84_fallback"


def llh_to_ecef(lat_deg, lon_deg, height_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    normal_radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (normal_radius + height_m) * cos_lat * cos_lon
    y = (normal_radius + height_m) * cos_lat * sin_lon
    z = (normal_radius * (1.0 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def ecef_to_enu(x, y, z, origin_lat_deg, origin_lon_deg, origin_height_m):
    ox, oy, oz = llh_to_ecef(origin_lat_deg, origin_lon_deg, origin_height_m)
    dx = x - ox
    dy = y - oy
    dz = z - oz

    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def llh_to_enu(lat_deg, lon_deg, height_m, origin_llh):
    if _pymap3d is not None:
        east, north, up = _pymap3d.geodetic2enu(
            lat_deg,
            lon_deg,
            height_m,
            origin_llh[0],
            origin_llh[1],
            origin_llh[2],
        )
        return float(east), float(north), float(up)
    x, y, z = llh_to_ecef(lat_deg, lon_deg, height_m)
    return ecef_to_enu(x, y, z, origin_llh[0], origin_llh[1], origin_llh[2])
