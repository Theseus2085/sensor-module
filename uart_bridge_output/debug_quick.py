"""
Quick diagnostic: test COM3 (Mega) and COM6 (Nucleo STLink) back-to-back.
Output goes to debug_output.txt for clean reading.
"""
import serial
import time
import sys

BAUD_RATE = 115200
OUTPUT_FILE = "debug_output.txt"

def test_port(port_name, f):
    msg = f"\n{'='*60}\n  TESTING: {port_name} at {BAUD_RATE} baud\n{'='*60}\n"
    print(msg, end='')
    f.write(msg)

    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
    except Exception as e:
        msg = f"  [ERROR] Could not open {port_name}: {e}\n"
        print(msg, end='')
        f.write(msg)
        return

    msg = f"  [OK] Port opened. Waiting 3s for bootloader...\n"
    print(msg, end='')
    f.write(msg)
    time.sleep(3)

    msg = f"  [READ] Reading for 10 seconds...\n"
    print(msg, end='')
    f.write(msg)

    start = time.time()
    total_bytes = 0
    total_lines = 0
    empty_reads = 0
    errors = 0

    try:
        while (time.time() - start) < 10:
            elapsed = time.time() - start
            try:
                raw = ser.read(256)
                if raw:
                    total_bytes += len(raw)
                    newlines = raw.count(b'\n')
                    total_lines += newlines
                    hex_str = ' '.join(f'{b:02X}' for b in raw)
                    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw)
                    line = f"  [{elapsed:6.2f}s] {len(raw):3d}B | HEX: {hex_str}\n"
                    line += f"           | ASCII: {ascii_str}\n"
                    print(line, end='')
                    f.write(line)
                else:
                    empty_reads += 1
            except serial.SerialException as e:
                errors += 1
                line = f"  [{elapsed:6.2f}s] [SERIAL ERROR #{errors}] {e}\n"
                print(line, end='')
                f.write(line)
                if errors >= 3:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    duration = time.time() - start
    summary = (
        f"\n  SUMMARY for {port_name}:\n"
        f"  Duration: {duration:.1f}s | Bytes: {total_bytes} | "
        f"Lines: {total_lines} | Empty reads: {empty_reads} | Errors: {errors}\n"
    )
    if total_bytes == 0:
        summary += "  >>> NO DATA RECEIVED <<<\n"
    else:
        summary += f"  >>> DATA OK ({total_bytes/duration:.1f} bytes/sec) <<<\n"
    print(summary, end='')
    f.write(summary)


def main():
    ports_to_test = ["COM3", "COM6"]
    
    with open(OUTPUT_FILE, 'w') as f:
        header = f"Serial Debug Run at {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        f.write(header)
        print(header, end='')
        
        for port in ports_to_test:
            test_port(port, f)
        
        msg = f"\nDone. Full output saved to {OUTPUT_FILE}\n"
        print(msg, end='')
        f.write(msg)


if __name__ == "__main__":
    main()
