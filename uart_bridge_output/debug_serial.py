"""
Debug script to diagnose serial connection issues between
  Elegoo Mega 2560 <-> STM32 Nucleo F446RE UART bridge.

This script tests the raw serial byte stream and reports
exactly what is (or isn't) coming through.
"""
import serial
import serial.tools.list_ports
import time
import sys

BAUD_RATE = 115200

def list_ports():
    """Show all available COM ports with details."""
    ports = serial.tools.list_ports.comports()
    print(f"\n{'='*60}")
    print(f"  AVAILABLE COM PORTS ({len(ports)} found)")
    print(f"{'='*60}")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device}")
        print(f"      Description : {port.description}")
        print(f"      HWID        : {port.hwid}")
        print(f"      VID:PID     : {port.vid}:{port.pid}" if port.vid else "")
        print(f"      Manufacturer: {port.manufacturer}")
        print()
    return ports

def test_port(port_name):
    """Open port and dump raw bytes for diagnosis."""
    print(f"\n{'='*60}")
    print(f"  TESTING: {port_name} at {BAUD_RATE} baud")
    print(f"{'='*60}")

    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            timeout=1.0,        # 1 second read timeout
            write_timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False        # Disable DTR to prevent Arduino reset
        )
    except serial.SerialException as e:
        print(f"  [ERROR] Could not open port: {e}")
        return
    except Exception as e:
        print(f"  [ERROR] Unexpected error opening port: {e}")
        return

    print(f"  [OK] Port opened successfully.")
    print(f"       is_open    = {ser.is_open}")
    print(f"       name       = {ser.name}")
    print(f"       baudrate   = {ser.baudrate}")
    print(f"       bytesize   = {ser.bytesize}")
    print(f"       parity     = {ser.parity}")
    print(f"       stopbits   = {ser.stopbits}")
    print(f"       timeout    = {ser.timeout}")
    print(f"       rtscts     = {ser.rtscts}")
    print(f"       dsrdtr     = {ser.dsrdtr}")

    # Wait for Arduino bootloader to finish (it resets on connect)
    print(f"\n  [WAIT] Waiting 3 seconds for Arduino bootloader reset...")
    time.sleep(3)

    print(f"\n  [READ] Reading raw bytes for 15 seconds...")
    print(f"         (Press Ctrl+C to stop early)")
    print(f"{'='*60}")

    start_time = time.time()
    total_bytes = 0
    total_lines = 0
    empty_reads = 0
    errors = 0

    try:
        while (time.time() - start_time) < 15:
            elapsed = time.time() - start_time
            try:
                # Read raw bytes (up to 256 at a time)
                raw = ser.read(256)
                
                if raw:
                    total_bytes += len(raw)
                    # Show hex dump and ascii
                    hex_str = ' '.join(f'{b:02X}' for b in raw)
                    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw)
                    
                    # Count newlines
                    newline_count = raw.count(b'\n')
                    total_lines += newline_count
                    
                    print(f"  [{elapsed:6.2f}s] {len(raw):3d} bytes | HEX: {hex_str[:80]}")
                    print(f"           {'':3s}       | ASCII: {ascii_str[:80]}")
                    if len(hex_str) > 80:
                        print(f"           {'':3s}       | (truncated, full length: {len(raw)} bytes)")
                else:
                    empty_reads += 1
                    # Only print empty read notification every 3rd time to reduce spam
                    if empty_reads % 3 == 1:
                        print(f"  [{elapsed:6.2f}s] --- no data (timeout, count: {empty_reads}) ---")
                        
            except serial.SerialException as e:
                errors += 1
                print(f"  [{elapsed:6.2f}s] [SERIAL ERROR #{errors}] {e}")
                if errors >= 5:
                    print(f"\n  [ABORT] Too many serial errors ({errors}). Stopping.")
                    break
            except Exception as e:
                errors += 1
                print(f"  [{elapsed:6.2f}s] [ERROR #{errors}] {type(e).__name__}: {e}")
                if errors >= 5:
                    print(f"\n  [ABORT] Too many errors ({errors}). Stopping.")
                    break

    except KeyboardInterrupt:
        print(f"\n  [STOP] Interrupted by user.")

    finally:
        ser.close()

    duration = time.time() - start_time
    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    print(f"  Duration     : {duration:.1f} seconds")
    print(f"  Total bytes  : {total_bytes}")
    print(f"  Total lines  : {total_lines}")
    print(f"  Empty reads  : {empty_reads}")
    print(f"  Errors       : {errors}")
    print(f"  Avg bytes/sec: {total_bytes/duration:.1f}" if duration > 0 else "")
    print()

    if total_bytes == 0:
        print("  [DIAGNOSIS] No data received at all!")
        print("  Possible causes:")
        print("    1. Wrong COM port selected (try STLink COM6 instead of Mega COM3)")
        print("    2. Wiring issue: Nucleo D1 (TX) must connect to Mega RX1 (Pin 19)")
        print("    3. Missing GND connection between boards")
        print("    4. Nucleo firmware not running (check LED heartbeat)")
        print("    5. Baud rate mismatch")
    elif errors > 0:
        print("  [DIAGNOSIS] Data received but with serial errors.")
        print("  This is typically a USB driver issue with CH340 clone chips.")
        print("  Try: different USB port, shorter cable, or reinstall CH340 driver.")
    else:
        print("  [DIAGNOSIS] Data received successfully!")
        print("  The serial connection is working properly.")

def main():
    ports = list_ports()
    if not ports:
        print("No COM ports found!")
        return

    # Ask which port to test
    print("Which port to test?")
    print("  - Enter the index number (e.g. 2 for COM3)")
    print("  - Or type 'all' to test all ports")
    choice = input("> ").strip()

    if choice.lower() == 'all':
        for port in ports:
            test_port(port.device)
    else:
        try:
            idx = int(choice)
            test_port(ports[idx].device)
        except (ValueError, IndexError):
            print(f"Invalid choice: {choice}")

if __name__ == "__main__":
    main()
