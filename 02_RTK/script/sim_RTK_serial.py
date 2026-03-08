import time
def nmea(payload):
    c=0
    for ch in payload: c ^= ord(ch)
    return f"${payload}*{c:02X}\r\n".encode("ascii")

lat, lon = 38.941292, -92.318846
while True:
    alat, alon = abs(lat), abs(lon)
    lat_dd = int(alat); lat_mm = (alat-lat_dd)*60
    lon_dd = int(alon); lon_mm = (alon-lon_dd)*60
    lat_s = f"{lat_dd:02d}{lat_mm:08.5f}"
    lon_s = f"{lon_dd:03d}{lon_mm:08.5f}"
    ns = "N" if lat>=0 else "S"
    ew = "E" if lon>=0 else "W"

    gga = nmea(f"GNGGA,123519,{lat_s},{ns},{lon_s},{ew},4,18,0.7,205.4,M,0.0,M,,")
    rmc = nmea(f"GNRMC,123519,A,{lat_s},{ns},{lon_s},{ew},1.20,87.3,050326,,,A")

    # rtk_bridge.py will read from /tmp/rtk_in, so we write to /tmp/rtk_out for testing
    # *****************************************************************************
    # To create the virtual serial ports, run the following command in the terminal before executing this script:
    # socat -d -d pty,raw,echo=0,link=/tmp/rtk_in pty,raw,echo=0,link=/tmp/rtk_out
    # *****************************************************************************
    with open('/tmp/rtk_out', 'wb', buffering=0) as f:
        f.write(gga); f.write(rmc)

    lon += 0.00001
    time.sleep(0.2)
