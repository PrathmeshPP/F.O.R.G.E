"""F.O.R.G.E — DJI Drone Telemetry & Geodetic Coordinate Conversion Module.

Extracts telemetry (WGS84 GPS latitude, longitude, ellipsoidal height, barometer pressure,
and orientation) from:
1. Embedded DJI MP4/MOV timed metadata tracks (`djmd` box with protobuf payloads).
2. Companion DJI SRT subtitle files.
3. Flight record logs.

Converts geodetic coordinates (WGS84 latitude, longitude, height) to local Cartesian
East-North-Up (ENU) coordinates in meters relative to a reference coordinate anchor.
"""

from __future__ import annotations

import re
import math
import struct
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple, Dict, Any

import numpy as np


# WGS-84 Ellipsoid Constants (EPSG:4979 / EPSG:4326)
WGS84_SEMI_MAJOR_AXIS_A = 6378137.0          # Semi-major axis (meters)
WGS84_FLATTENING_F = 1.0 / 298.257223563      # Reciprocal flattening
WGS84_SEMI_MINOR_AXIS_B = WGS84_SEMI_MAJOR_AXIS_A * (1.0 - WGS84_FLATTENING_F)
WGS84_ECCENTRICITY_SQ_E2 = (WGS84_SEMI_MAJOR_AXIS_A**2 - WGS84_SEMI_MINOR_AXIS_B**2) / (WGS84_SEMI_MAJOR_AXIS_A**2)


@dataclass
class GPSTelemetryRecord:
    """Represents a time-synchronized drone telemetry observation."""
    sample_index: int
    timestamp_sec: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    barometer_pa: Optional[float] = None
    quaternion_wxyz: Optional[Tuple[float, float, float, float]] = None
    source_type: str = "djmd_protobuf"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    """
    Converts geodetic coordinates (WGS84) to Earth-Centered, Earth-Fixed (ECEF) coordinates in meters.

    Args:
        lat_deg: Latitude in decimal degrees [-90, 90].
        lon_deg: Longitude in decimal degrees [-180, 180].
        alt_m: Ellipsoidal height in meters above WGS84 reference ellipsoid.

    Returns:
        (X, Y, Z) in ECEF meters.
    """
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)

    # Prime vertical radius of curvature
    N = WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(1.0 - WGS84_ECCENTRICITY_SQ_E2 * sin_phi**2)

    X = (N + alt_m) * cos_phi * cos_lam
    Y = (N + alt_m) * cos_phi * sin_lam
    Z = ((1.0 - WGS84_ECCENTRICITY_SQ_E2) * N + alt_m) * sin_phi

    return X, Y, Z


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float
) -> Tuple[float, float, float]:
    """
    Converts geodetic coordinates to local East-North-Up (ENU) coordinates in meters
    relative to a local reference origin.

    Args:
        lat_deg: Target latitude in decimal degrees.
        lon_deg: Target longitude in decimal degrees.
        alt_m: Target height in meters.
        ref_lat_deg: Origin reference latitude in decimal degrees.
        ref_lon_deg: Origin reference longitude in decimal degrees.
        ref_alt_m: Origin reference height in meters.

    Returns:
        (East, North, Up) coordinates in meters relative to reference origin.
    """
    X, Y, Z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    X0, Y0, Z0 = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)

    dX = X - X0
    dY = Y - Y0
    dZ = Z - Z0

    phi0 = math.radians(ref_lat_deg)
    lam0 = math.radians(ref_lon_deg)
    sin_p = math.sin(phi0)
    cos_p = math.cos(phi0)
    sin_l = math.sin(lam0)
    cos_l = math.cos(lam0)

    # Rotation from ECEF to local ENU
    E = -sin_l * dX + cos_l * dY
    N = -sin_p * cos_l * dX - sin_p * sin_l * dY + cos_p * dZ
    U =  cos_p * cos_l * dX + cos_p * sin_l * dY + sin_p * dZ

    return E, N, U


def enu_to_geodetic(
    east_m: float,
    north_m: float,
    up_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float
) -> Tuple[float, float, float]:
    """
    Converts local ENU coordinates in meters back to geodetic WGS84 coordinates.
    """
    phi0 = math.radians(ref_lat_deg)
    lam0 = math.radians(ref_lon_deg)
    sin_p = math.sin(phi0)
    cos_p = math.cos(phi0)
    sin_l = math.sin(lam0)
    cos_l = math.cos(lam0)

    # Inverse rotation from ENU to ECEF
    dX = -sin_l * east_m - sin_p * cos_l * north_m + cos_p * cos_l * up_m
    dY =  cos_l * east_m - sin_p * sin_l * north_m + cos_p * sin_l * up_m
    dZ =  cos_p * north_m + sin_p * up_m

    X0, Y0, Z0 = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    X = X0 + dX
    Y = Y0 + dY
    Z = Z0 + dZ

    # Bowring's method for ECEF to geodetic
    p = math.sqrt(X**2 + Y**2)
    theta = math.atan2(Z * WGS84_SEMI_MAJOR_AXIS_A, p * WGS84_SEMI_MINOR_AXIS_B)
    e_prime_sq = (WGS84_SEMI_MAJOR_AXIS_A**2 - WGS84_SEMI_MINOR_AXIS_B**2) / (WGS84_SEMI_MINOR_AXIS_B**2)

    lat = math.atan2(
        Z + e_prime_sq * WGS84_SEMI_MINOR_AXIS_B * (math.sin(theta)**3),
        p - WGS84_ECCENTRICITY_SQ_E2 * WGS84_SEMI_MAJOR_AXIS_A * (math.cos(theta)**3)
    )
    lon = math.atan2(Y, X)

    sin_lat = math.sin(lat)
    N = WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(1.0 - WGS84_ECCENTRICITY_SQ_E2 * sin_lat**2)
    alt = p / math.cos(lat) - N

    return math.degrees(lat), math.degrees(lon), alt


def extract_dji_mp4_telemetry(mp4_path: str) -> List[GPSTelemetryRecord]:
    """
    Parses the timed metadata track (`djmd`) inside a DJI MP4 container, extracting
    per-frame protobuf telemetry payloads.

    Args:
        mp4_path: Path to the DJI video file.

    Returns:
        List of GPSTelemetryRecord objects extracted from the video container.
    """
    path = Path(mp4_path)
    if not path.is_file():
        raise FileNotFoundError(f"Video file not found: {mp4_path}")

    with open(path, "rb") as f:
        file_size = f.seek(0, 2)
        f.seek(0)

        # 1. Parse top-level MP4 boxes to find 'moov'
        moov_offset, moov_size = None, None
        while f.tell() < file_size:
            pos = f.tell()
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            sz, btype = struct.unpack(">I4s", hdr)
            header_len = 8
            if sz == 1:
                sz = struct.unpack(">Q", f.read(8))[0]
                header_len = 16
            elif sz == 0:
                sz = file_size - pos

            if btype == b"moov":
                moov_offset, moov_size = pos, sz
                break
            f.seek(pos + sz)

        if not moov_offset:
            return []

        def parse_sub_boxes(start: int, end: int) -> List[Tuple[bytes, int, int, int]]:
            f.seek(start)
            boxes = []
            while f.tell() < end:
                pos = f.tell()
                hdr = f.read(8)
                if len(hdr) < 8:
                    break
                sz, btype = struct.unpack(">I4s", hdr)
                hlen = 8
                if sz == 1:
                    sz = struct.unpack(">Q", f.read(8))[0]
                    hlen = 16
                elif sz == 0:
                    sz = end - pos
                boxes.append((btype, pos, sz, hlen))
                f.seek(pos + sz)
            return boxes

        # 2. Find trak containing 'djmd'
        moov_boxes = parse_sub_boxes(moov_offset + 8, moov_offset + moov_size)
        traks = [b for b in moov_boxes if b[0] == b"trak"]

        djmd_trak = None
        for tr in traks:
            f.seek(tr[1])
            data = f.read(min(tr[2], 65536))
            if b"djmd" in data:
                djmd_trak = tr
                break

        if not djmd_trak:
            return []

        # 3. Locate stbl inside djmd_trak -> mdia -> minf -> stbl
        tr_boxes = parse_sub_boxes(djmd_trak[1] + djmd_trak[3], djmd_trak[1] + djmd_trak[2])
        mdia = next((b for b in tr_boxes if b[0] == b"mdia"), None)
        if not mdia:
            return []
        mdia_boxes = parse_sub_boxes(mdia[1] + mdia[3], mdia[1] + mdia[2])

        # Get timescale from mdhd
        mdhd = next((b for b in mdia_boxes if b[0] == b"mdhd"), None)
        timescale = 60000.0  # default fallback
        if mdhd:
            f.seek(mdhd[1] + mdhd[3])
            version = struct.unpack(">B", f.read(1))[0]
            f.read(3)  # flags
            if version == 1:
                f.read(16)  # creation & modification time (64-bit each)
                timescale = float(struct.unpack(">I", f.read(4))[0])
            else:
                f.read(8)   # creation & modification time (32-bit each)
                timescale = float(struct.unpack(">I", f.read(4))[0])

        minf = next((b for b in mdia_boxes if b[0] == b"minf"), None)
        if not minf:
            return []
        minf_boxes = parse_sub_boxes(minf[1] + minf[3], minf[1] + minf[2])
        stbl = next((b for b in minf_boxes if b[0] == b"stbl"), None)
        if not stbl:
            return []
        stbl_boxes = parse_sub_boxes(stbl[1] + stbl[3], stbl[1] + stbl[2])

        # 4. Extract sample sizes (stsz) and chunk offsets (stco or co64)
        stsz = next((b for b in stbl_boxes if b[0] == b"stsz"), None)
        stco = next((b for b in stbl_boxes if b[0] == b"stco"), None)
        co64 = next((b for b in stbl_boxes if b[0] == b"co64"), None)

        if not stsz or (not stco and not co64):
            return []

        f.seek(stsz[1] + stsz[3] + 4)  # skip version/flags
        sample_size, sample_count = struct.unpack(">II", f.read(8))
        if sample_size != 0:
            sample_sizes = [sample_size] * sample_count
        else:
            sample_sizes = [struct.unpack(">I", f.read(4))[0] for _ in range(sample_count)]

        if stco:
            f.seek(stco[1] + stco[3] + 4)
            chunk_count = struct.unpack(">I", f.read(4))[0]
            chunk_offsets = [struct.unpack(">I", f.read(4))[0] for _ in range(chunk_count)]
        else:
            f.seek(co64[1] + co64[3] + 4)
            chunk_count = struct.unpack(">I", f.read(4))[0]
            chunk_offsets = [struct.unpack(">Q", f.read(8))[0] for _ in range(chunk_count)]

        num_records = min(sample_count, chunk_count)
        records: List[GPSTelemetryRecord] = []

        # Read samples
        for idx in range(num_records):
            f.seek(chunk_offsets[idx])
            s_data = f.read(sample_sizes[idx])

            # In djmd protobuf:
            # Tag 0x11 is field 2 (fixed64 double, latitude in radians)
            # Tag 0x19 is field 3 (fixed64 double, longitude in radians)
            p11 = s_data.find(b"\x11")
            lat_deg, lon_deg = None, None
            height_m = None
            baro_pa = None
            quat = None

            while p11 != -1:
                if p11 + 9 < len(s_data) and s_data[p11 + 9] == 0x19 and p11 + 18 <= len(s_data):
                    lat_r = struct.unpack("<d", s_data[p11 + 1 : p11 + 9])[0]
                    lon_r = struct.unpack("<d", s_data[p11 + 10 : p11 + 18])[0]
                    if 0.0 < lat_r < 1.57 and 0.0 < lon_r < 3.14159:
                        lat_deg = math.degrees(lat_r)
                        lon_deg = math.degrees(lon_r)

                        # Height follows as varint tag 0x10 (field 2) in millimeters
                        p = p11 + 18
                        if p < len(s_data) and s_data[p] == 0x10:
                            p += 1
                            val, shift = 0, 0
                            while p < len(s_data):
                                b = s_data[p]
                                p += 1
                                val |= (b & 0x7F) << shift
                                shift += 7
                                if not (b & 0x80):
                                    break
                            height_m = val / 1000.0  # mm to meters
                        break
                p11 = s_data.find(b"\x11", p11 + 1)

            # Search for barometer (Pa)
            pos_baro = s_data.find(b"\x2a\x07\x0d")
            if pos_baro != -1 and pos_baro + 7 <= len(s_data):
                baro_pa = struct.unpack("<f", s_data[pos_baro + 3 : pos_baro + 7])[0]

            # Search for quaternion
            for q_offset in range(len(s_data) - 16):
                try:
                    q = struct.unpack("<4f", s_data[q_offset : q_offset + 16])
                    norm_sq = sum(x * x for x in q)
                    if 0.999 < norm_sq < 1.001 and all(-1.01 <= x <= 1.01 for x in q):
                        quat = q
                        break
                except Exception:
                    pass

            if lat_deg is not None and lon_deg is not None and height_m is not None:
                # Sample 0 is track dictionary header; sample 1 corresponds to frame 0
                sample_time = max(0.0, (idx - 1) / 59.940) if idx >= 1 else 0.0
                records.append(
                    GPSTelemetryRecord(
                        sample_index=idx,
                        timestamp_sec=round(sample_time, 4),
                        latitude_deg=lat_deg,
                        longitude_deg=lon_deg,
                        altitude_m=height_m,
                        barometer_pa=baro_pa,
                        quaternion_wxyz=quat,
                        source_type="djmd_protobuf"
                    )
                )

        return records


def extract_dji_srt_telemetry(srt_path: str) -> List[GPSTelemetryRecord]:
    """
    Parses a DJI companion .SRT subtitle file containing GPS and barometer records.

    Args:
        srt_path: Path to the .SRT file.

    Returns:
        List of GPSTelemetryRecord objects extracted from the subtitle text.
    """
    path = Path(srt_path)
    if not path.is_file():
        raise FileNotFoundError(f"SRT file not found: {srt_path}")

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    entries: List[GPSTelemetryRecord] = []
    pattern = re.compile(
        r"(\d+)\s*\n(\d{2}):(\d{2}):(\d{2})[,.](\d{3})\s*-->\s*\d{2}:\d{2}:\d{2}[,.]\d{3}\s*\n(.*?)(?=\n\s*\n|\Z)",
        re.DOTALL
    )

    for m in pattern.finditer(content):
        idx = int(m.group(1))
        h, mn, s, ms = int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
        timestamp_sec = h * 3600.0 + mn * 60.0 + s + ms / 1000.0

        body = m.group(6)
        # Match GPS(lon, lat, alt) or GPS (lon, lat)
        gps_m = re.search(r"GPS\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)(?:\s*,\s*([-\d.]+))?\s*\)", body)
        baro_m = re.search(r"BAROMETER\s*:\s*([-\d.]+)", body)

        if gps_m:
            lon = float(gps_m.group(1))
            lat = float(gps_m.group(2))
            alt = float(gps_m.group(3)) if gps_m.group(3) else 0.0
            baro = float(baro_m.group(1)) if baro_m else None

            entries.append(
                GPSTelemetryRecord(
                    sample_index=idx,
                    timestamp_sec=round(timestamp_sec, 4),
                    latitude_deg=lat,
                    longitude_deg=lon,
                    altitude_m=alt,
                    barometer_pa=baro,
                    source_type="dji_srt"
                )
            )

    return entries


def extract_dji_telemetry(source_path: str) -> List[GPSTelemetryRecord]:
    """
    General entrypoint: extracts DJI drone telemetry from either an MP4 video or an SRT file.
    """
    p = Path(source_path)
    if p.suffix.lower() == ".srt":
        return extract_dji_srt_telemetry(str(p))
    elif p.suffix.lower() in [".mp4", ".mov"]:
        # First check if a companion .SRT file exists in the same folder
        candidate_srt = p.with_suffix(".SRT")
        if not candidate_srt.is_file():
            candidate_srt = p.with_suffix(".srt")

        if candidate_srt.is_file():
            recs = extract_dji_srt_telemetry(str(candidate_srt))
            if len(recs) > 0:
                return recs

        # Fall back to parsing the embedded djmd metadata track
        return extract_dji_mp4_telemetry(str(p))
    else:
        raise ValueError(f"Unsupported telemetry source format: {source_path}")


def match_telemetry_to_frame_timestamps(
    telemetry_records: List[GPSTelemetryRecord],
    frame_timestamps_sec: List[float],
    max_time_tolerance_sec: float = 0.5
) -> List[Optional[GPSTelemetryRecord]]:
    """
    Associates each extracted frame timestamp with the closest matching telemetry record.

    Args:
        telemetry_records: List of sorted telemetry observations.
        frame_timestamps_sec: Target timestamps (in seconds) for each reconstructed frame.
        max_time_tolerance_sec: Maximum permissible discrepancy between frame and GPS fix.

    Returns:
        List of matched GPSTelemetryRecord objects (or None if no fix is within tolerance).
    """
    if len(telemetry_records) == 0:
        return [None] * len(frame_timestamps_sec)

    tel_times = np.array([r.timestamp_sec for r in telemetry_records])
    matched: List[Optional[GPSTelemetryRecord]] = []

    for t_frame in frame_timestamps_sec:
        idx = int(np.argmin(np.abs(tel_times - t_frame)))
        dt = abs(tel_times[idx] - t_frame)
        if dt <= max_time_tolerance_sec:
            matched.append(telemetry_records[idx])
        else:
            matched.append(None)

    return matched
