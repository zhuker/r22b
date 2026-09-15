# Bumble BLE Service

This is a rewrite of `hello.py` using [Google Bumble](https://github.com/google/bumble) - a pure Python BLE stack.

## Features

- **RealDash CAN Data Streaming**: Temperature sensor + TPMS data
- **H.264 Video Streaming**: Packetized video frames over BLE
- **TPMS Scanner**: Background scanning for tire pressure sensors
- **Pure Python**: No D-Bus dependencies for BLE

## Advantages of Bumble

1. **No BlueZ D-Bus**: Direct HCI access without D-Bus complexity
2. **Cross-platform**: Works on Linux, macOS, Windows
3. **Modern async/await**: Built for asyncio from the ground up
4. **Better control**: Full control over BLE stack behavior
5. **No advertisement errors**: Direct control over advertising packets

## Installation

```bash
pip install bumble
```

Or install all requirements:
```bash
pip install -r requirements.txt
```

## Usage

The transport needs to be specified. Options:
- `usb:0` - USB Bluetooth adapter
- `hci:0` - Built-in Bluetooth (requires permissions)
- `android-emulator` - Android emulator

Edit line 536 in `bumble_service.py`:
```python
async with await open_transport_or_link('usb:0') as (hci_source, hci_sink):
```

Run:
```bash
sudo python3 bumble_service.py
```

## BLE Service Structure

**NUS Service**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- **NUS TX** (Notify): RealDash CAN data - `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- **NUS RX** (Write): Unused - `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- **H.264 Stream** (Notify): Video packets - `6E400012-B5A3-F393-E0A9-E50E24DCCA9E`
- **H.264 Control** (Write): Stream control - `6E400013-B5A3-F393-E0A9-E50E24DCCA9E`

## H.264 Streaming Control

Write commands to the H.264 Control characteristic:
- `START` - Begin streaming frames
- `STOP` - Pause streaming
- `RESET` - Reset to first frame

## Differences from hello.py

- Uses Bumble instead of bluezero
- Direct HCI access instead of BlueZ D-Bus
- Pure asyncio implementation (no threading for BLE)
- Better error handling
- Cleaner characteristic subscription handling

## Troubleshooting

**Permission denied on HCI:**
```bash
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(which python3)
```

**Find your transport:**
```bash
bumble-scan  # List available transports
```

**USB adapter not found:**
- Check `lsusb` for Bluetooth adapters
- Try `usb:0`, `usb:1`, etc.

## Performance

- RealDash updates: 1 Hz
- H.264 streaming: ~30 FPS
- Packet size: Up to 500 bytes
- STAP-A aggregation for small NAL units
- FU-A fragmentation for large NAL units
