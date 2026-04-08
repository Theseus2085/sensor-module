import serial
import serial.tools.list_ports
import csv
import time
import os

BAUD_RATE = 115200
# Default to STLink VCP if available (Nucleo outputs printf here)
PREFERRED_PORT_KEYWORDS = ["STLink", "STMicroelectronics"]

def find_port():
    """Auto-detect the STLink VCP port, or let the user choose."""
    ports = serial.tools.list_ports.comports()
    
    # Try to auto-detect STLink VCP
    for port in ports:
        for keyword in PREFERRED_PORT_KEYWORDS:
            if keyword.lower() in port.description.lower():
                print(f"Auto-detected STLink VCP: {port.device} ({port.description})")
                return port.device
    
    # Fall back to manual selection
    print("Available ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    
    if not ports:
        print("No serial ports found!")
        return None
        
    choice = input("Select port index: ")
    try:
        return ports[int(choice)].device
    except:
        return None

def main():
    port = find_port()
    if not port:
        return
        
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    csv_filename = f"sensor_measurements_{timestamp}.csv"
    
    print(f"Connecting to {port} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(
            port=port,
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
        print(f"Error opening port: {e}")
        return

    # Wait for Nucleo to finish booting after a possible reset
    print("Waiting 2s for board to stabilize...")
    time.sleep(2)

    print(f"Connected! Logging to {csv_filename}")
    print("Press Ctrl+C to stop.\n")
    print("-" * 50)
    
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Timestamp", "Sensor1_mm", "Sensor2_mm"])
        
        try:
            while True:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue
                    
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                    
                # Print everything to terminal
                print(line)
                
                # Log measurement lines (format: "1.7500,1.7500") to CSV
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        try:
                            # Verify both parts are valid floats
                            float(parts[0].strip())
                            float(parts[1].strip())
                            current_time = time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"
                            csv_writer.writerow([current_time, parts[0].strip(), parts[1].strip()])
                            csvfile.flush()
                        except ValueError:
                            pass  # Not a measurement line, skip CSV logging
                
        except KeyboardInterrupt:
            print("\n\nLogging stopped by user.")
        finally:
            ser.close()
            print(f"File saved: {os.path.abspath(csv_filename)}")

if __name__ == "__main__":
    main()
