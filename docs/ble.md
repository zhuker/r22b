# BLE on r22b

How Bluetooth LE is set up on the Pi prototype, based on the system state,
shell history (user and root) and code as found on 2026-09-14. The Pi was not
modified while collecting this.

## Short version

- **Nothing BLE-related runs at boot.** There are no custom systemd units,
  user units, cron jobs or rc.local entries. The only Bluetooth service is
  stock `bluetooth.service` (bluetoothd), enabled with default config.
- Every BLE service was **started by hand** from a shell, as root.
- There were two generations of the GATT peripheral:

| | 1. bluezero (Dec 2025) | 2. Bumble (Dec 29 2025 – Jan 3 2026) |
|---|---|---|
| Code | `experiments/legacy/bluezero_service.py` (was `hello.py`) | `r22b/ble_service.py` (was `bumble_service.py`) |
| Stack | BlueZ over D-Bus (bluezero) | Bumble, raw HCI socket, owns the controller |
| bluetoothd | must be running | **must be stopped**, `hci0` down |
| Advertised name | `STI_Diff_Sensor` | `22B_Sensors` |
| Address | controller's public `E4:5F:01:9C:1C:9D` | hard-coded static random `F0:F1:F2:F3:F4:F5` |
| TPMS scanning (Bleak) | works (goes through bluetoothd) | **does not work**: Bleak needs bluetoothd, so TPMS values stay 0 |
| H.264 streaming | sample files, `set_value` + `time.sleep` | live USB camera via PyAV/libx264 |

## Running the Bumble service (as it was run)

From root's history, repeated dozens of times:

```sh
sudo systemctl stop bluetooth
sudo hciconfig hci0 down
cd ~/git/High-Pricision_AD_HAT/python && . .venv/bin/activate
python3 bumble_service.py        # as root (sudo su first)
```

In this repo that becomes `sudo .venv/bin/python -m r22b.ble_service` from the
repo root. Transport is `hci-socket:0` (the Pi 4's onboard CYW43455 on UART).

`sudo systemctl start bluetooth` gives the controller back to BlueZ.

Attempts to avoid root, none of which stuck: `capsh` with `cap_net_admin`,
and the old README suggesting `setcap cap_net_raw,cap_net_admin+eip` on
python3. `getcap` shows no capabilities set on anything today.

## GATT table

One primary service, the Nordic UART Service UUID, with two extra
characteristics tacked on. Same UUIDs in both generations.

| Characteristic | UUID | Props | Use |
|---|---|---|---|
| NUS TX | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | notify | RealDash CAN frames |
| NUS RX | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | write, write w/o resp | unused |
| H.264 stream | `6E400012-B5A3-F393-E0A9-E50E24DCCA9E` | notify | H.264 packets |
| H.264 control | `6E400013-B5A3-F393-E0A9-E50E24DCCA9E` | write, write w/o resp | ASCII `START` / `STOP` / `RESET` |

Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`. The Bumble version
advertises it in the advertising data and puts the name in the scan response.
It also advertises appearance "earbud", probably left over from a Bumble example.

### RealDash frames (NUS TX)

RealDash CAN "0x44" framing, 16 bytes each, five per cycle
(`r22b/realdash.py`, channel definitions in `data/realdash/realdash_5.xml`):

| CAN id | Payload (little-endian) |
|---|---|
| `0xC80` | int16 rear diff temp, °C × 10 |
| `0xC81`–`0xC84` (FL, FR, RL, RR) | uint16 pressure bar × 100, int16 temp °C × 10, uint8 leaking, uint8 battery-low |

### H.264 stream

Subscribing to the stream characteristic starts streaming immediately
(`START` isn't actually required). Each encoded frame gets an access unit
delimiter prepended, then is split with RTP-style H.264 payloading
(RFC 6184): single NAL, STAP-A for small NALs, FU-A fragments, max 500 bytes
per notification (`r22b/video/packetizer.py`). `r22b/video/depacketize.py`
reverses it. Camera: `/dev/video0`, 640×360@30, libx264 `tune=zerolatency`,
no B-frames, GOP 600, 300 kbps.

Commit history: `h264 packetize/depacketize` → `bumble streaming service` →
`camera frames streamed ok` (Dec 30 2025). The phone side lives in dash22b.

## Things tried along the way

- **BlueZ config edits (Dec 27 2025).** History shows edits to
  `bluetooth.service` (through its `bluetooth.target.wants` symlink) and
  `/etc/bluetooth/main.conf`, each followed by a daemon-reload and restart.
  Neither file carries a change today: `main.conf` is all comments and
  `ExecStart` is plain `bluetoothd`. The unit file's mtime is still Dec 27, so
  the edit was probably saved and later reverted. What the edit was isn't
  recoverable (guess: `--experimental` for the bluezero MTU/advertising
  work).
- **MTU probing** in the bluezero service: `log_negotiated_mtu()` reads
  `Device1.MTU` over D-Bus, and `probe_notification_limit()` tries decreasing
  payload sizes. Note that bluezero's `set_value` doesn't raise when a
  notification is too long, so the probe likely always "succeeds" at 100
  bytes (inference, not tested).
- **`bumble-bench`** throughput runs from a separate venv at `~/tmp/venv`
  (peripheral/central, `send` scenario, 256/512-byte packets, GATT and
  `l2cap-server` modes), plus `btmgmt phy` to check LE PHYs. The results
  weren't saved.
- **Raw scans** with `hcitool lescan --duplicates` to find the TPMS sensors.
  The decoded capture is `data/captures/tpms-scanlog.txt`: sensors
  `TPMS1_5038D5`, `TPMS2_5038EE`, `TPMS3_503941`, ...

## Known issues in `r22b/ble_service.py`

These are carried over unchanged from the Pi:

1. **TPMS is dead in Bumble mode** (see the table above). Fix options: scan
   with Bumble itself on the same controller, or put a second BT adapter on
   bluetoothd.
2. **RealDash frames aren't 1 Hz.** The main loop's `sleep(1)` is commented
   out, so while subscribed it sends 5 frames every ~1 ms loop pass, and does a
   blocking SPI ADC read each time.
3. **Camera capture and x264 encode run synchronously** on the asyncio loop,
   and the camera source prints a hex dump per encoded packet.
4. The static address `F0:F1:F2:F3:F4:F5` is the same placeholder as in the
   Bumble examples. Any other Bumble device using it will collide.
