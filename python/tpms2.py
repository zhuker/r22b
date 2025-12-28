from bleak import BleakScanner
import asyncio
import struct

def decode_tpms(raw_bytes):
    # Pressure is Bytes 6-9 (Little Endian uint32)
    pressure_raw = struct.unpack('<I', raw_bytes[6:10])[0]
    pressure_psi = (pressure_raw / 1000) * 0.145038

    # Temp is Bytes 10-13 (Little Endian uint32)
    temp_raw = struct.unpack('<I', raw_bytes[10:14])[0]
    temp_c = temp_raw / 100

    # Battery is typically at index 14
    battery_raw = raw_bytes[14]
    
    # If the value is > 50, it's likely a Percentage (0-100)
    # If the value is ~30, it's likely Voltage (Value / 10)
    if battery_raw > 50:
        battery_status = f"{battery_raw}%"
    else:
        battery_status = f"{battery_raw / 10.0}V"

    return pressure_psi, temp_c, battery_status

def detection_callback(device, advertisement_data):
#    print(device, advertisement_data)
    if device.name and "TPMS" in device.name:
        p,t,b = decode_tpms(advertisement_data.manufacturer_data[256])
        #print("\nFOUND:", device.name, device.address)
        #print("Service Data:", advertisement_data, advertisement_data.manufacturer_data)
        print(f"{device.name} Pressure: {p:.2f} PSI | Temp: {t:.2f} °C | Battery {b}%")


async def main():
    scanner = BleakScanner(detection_callback)
    await scanner.start()
    print("Scanning... (Ctrl+C to stop)")
    while True:
        await asyncio.sleep(1)

asyncio.run(main())
