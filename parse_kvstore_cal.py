"""
Parse STM32 flash dump to extract CalibrationPoint data from mbed KVStore.

The calibration table is:
  CalibrationPoint calibration_tables[2][3]
  struct CalibrationPoint { uint16_t raw_adc; float diameter_mm; };

On ARM Cortex-M4, sizeof(CalibrationPoint) = 8 bytes (2 + 2 padding + 4).
Total table size = 2 * 3 * 8 = 48 bytes.

We search for the known float patterns of the calibration diameters
(1.65, 1.75, 1.85) to locate the table in the dump.
"""

import struct
import sys

FLASH_DUMP = r"c:\Users\groll1960\projects\sensor-module\flash_kvstore.bin"
BASE_ADDR = 0x08040000

# Known calibration diameters as IEEE 754 floats (little-endian)
KNOWN_FLOATS = {
    1.65: struct.pack('<f', 1.65),
    1.75: struct.pack('<f', 1.75),
    1.85: struct.pack('<f', 1.85),
}

def parse_calibration_point(data, offset):
    """Parse one CalibrationPoint (8 bytes with padding)."""
    raw_adc = struct.unpack_from('<H', data, offset)[0]      # uint16_t at +0
    diameter = struct.unpack_from('<f', data, offset + 4)[0]  # float at +4 (after 2 bytes padding)
    return raw_adc, diameter

def try_parse_table(data, offset):
    """Try to parse a full calibration_tables[2][3] at the given offset."""
    points = []
    for s in range(2):
        sensor_points = []
        for p in range(3):
            pos = offset + (s * 3 + p) * 8
            if pos + 8 > len(data):
                return None
            raw_adc, diameter = parse_calibration_point(data, pos)
            sensor_points.append((raw_adc, diameter))
        points.append(sensor_points)
    return points

def is_plausible_table(table):
    """Check if a parsed table looks like valid calibration data."""
    for s in range(2):
        for p in range(3):
            raw_adc, diameter = table[s][p]
            # ADC values should be in 0..4095 range (12-bit)
            if raw_adc > 4095:
                return False
            # Diameters should be in reasonable range (1.0 - 3.0 mm)
            if diameter < 1.0 or diameter > 3.0:
                return False
        # ADC values should be monotonically increasing
        if not (table[s][0][0] < table[s][1][0] < table[s][2][0]):
            return False
    return True

def main():
    with open(FLASH_DUMP, 'rb') as f:
        data = f.read()

    print(f"Flash dump size: {len(data)} bytes")
    print(f"Base address: 0x{BASE_ADDR:08X}")
    print()

    # Strategy 1: Search for known float 1.75 (the middle calibration point)
    # and try to interpret surrounding data as the calibration table
    target = KNOWN_FLOATS[1.75]
    matches = []
    for i in range(len(data) - 4):
        if data[i:i+4] == target:
            matches.append(i)

    print(f"Found {len(matches)} occurrences of float 1.75 in flash dump")

    # For each match, try to interpret it as the middle point of a sensor's
    # calibration (offset -8 for point 0, this point is point 1, +8 for point 2)
    found_tables = []
    for match_offset in matches:
        # The float is at +4 within the CalibrationPoint struct
        # So the struct start is at match_offset - 4
        # If this is point 1 (index 1), table start is struct_start - 8
        struct_start = match_offset - 4
        table_start_as_p1 = struct_start - 8  # This is point 1 of sensor 0

        if table_start_as_p1 >= 0:
            table = try_parse_table(data, table_start_as_p1)
            if table and is_plausible_table(table):
                found_tables.append((table_start_as_p1, table))

        # Also try if this is point 1 of sensor 1 (offset - 3*8 - 8)
        table_start_as_s1p1 = struct_start - 3 * 8 - 8
        if table_start_as_s1p1 >= 0:
            table = try_parse_table(data, table_start_as_s1p1)
            if table and is_plausible_table(table):
                found_tables.append((table_start_as_s1p1, table))

    # Deduplicate by offset
    seen = set()
    unique_tables = []
    for offset, table in found_tables:
        if offset not in seen:
            seen.add(offset)
            unique_tables.append((offset, table))

    if not unique_tables:
        # Strategy 2: Brute force - try every offset
        print("No matches via float search, trying brute force scan...")
        for i in range(len(data) - 48):
            table = try_parse_table(data, i)
            if table and is_plausible_table(table):
                unique_tables.append((i, table))

    if not unique_tables:
        print("\nERROR: Could not find calibration data in flash dump.")
        print("The KVStore area may be empty or using a different format.")
        sys.exit(1)

    print(f"\nFound {len(unique_tables)} valid calibration table(s):\n")

    for idx, (offset, table) in enumerate(unique_tables):
        flash_addr = BASE_ADDR + offset
        print(f"=== Table {idx+1} at flash dump offset 0x{offset:06X} (flash addr 0x{flash_addr:08X}) ===")
        print()
        print("// --- Copy-paste this into main.cpp (line ~95) ---")
        print("CalibrationPoint calibration_tables[2][3] = {")
        for s in range(2):
            print(f"  {{ // Sensor {s+1}")
            for p in range(3):
                raw_adc, diameter = table[s][p]
                comma = "," if p < 2 else ""
                print(f"    {{{raw_adc}, {diameter:.2f}f}}{comma}")
            comma = "," if s < 1 else ""
            print(f"  }}{comma}")
        print("};")
        print()
        print("Raw values:")
        for s in range(2):
            for p in range(3):
                raw_adc, diameter = table[s][p]
                print(f"  Sensor {s+1}, Point {p+1}: raw_adc={raw_adc}, diameter={diameter:.4f} mm")
        print()

if __name__ == "__main__":
    main()
