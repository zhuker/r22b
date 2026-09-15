# r22b system setup

State of the Pi as read on 2026-09-14 (read-only). The customized config files
are copied under `system/`, mirroring their paths on the Pi.

## Hardware

- Raspberry Pi 4 Model B Rev 1.5, 8 GB, 64 GB SD card (6.5 GB used)
- Waveshare High-Precision AD HAT (ADS1263, SPI)
  - IN0: AEM 30-2012 NTC temp sensor, 2.2 kΩ pull-up (rear diff temp)
  - IN1: AEM 30-2130-150 pressure sensor, 0.5–4.5 V = 0–150 psi
    (`experiments/adc_probe.py`; what it measures isn't recorded)
- Onboard CYW43455 Bluetooth on UART (`hci0`, `E4:5F:01:9C:1C:9D`)
- Used over USB at various times (nothing attached now): UVC camera
  `/dev/video0`, SSM K-line cable `/dev/ttyUSB0` (VAG-COM style) or Tactrix
  `/dev/ttyACM0`

## OS

- Debian 13 (trixie) Raspberry Pi OS, kernel `6.12.47+rpt-rpi-v8`, aarch64
- Python 3.13, project venv at `python/.venv` in the old checkout
- Hostname `r22b`, user `zhukov`

## Boot and services (tuned for fast boot)

From `bash_history`, in order:

- **Networking:** NetworkManager installed then removed; `systemd-networkd`
  with a static `eth0` 192.168.1.50/24, gateway and DNS 192.168.1.254
  (`system/etc/systemd/network/10-eth0.network`). `wpa_supplicant` disabled,
  cloud-init purged, `netplan-ovs-cleanup` disabled.
- **Wi-Fi is off in firmware:** `dtoverlay=disable-wifi` in `config.txt`.
  A Wi-Fi transport (gateway plan Phase 4) needs that line removed.
- **Disabled or masked:** avahi-daemon, triggerhappy, rpi-eeprom-update,
  apt-daily(-upgrade) service and timer, man-db.timer, console-setup,
  keyboard-setup, e2scrub_reap, systemd-binfmt, rpi-resize-swap-file.
  Swap off (`dphys-swapfile swapoff`). brltty removed (it grabs USB serial
  adapters).
- **journald:** `Storage=volatile`, so logs don't survive a reboot.
- **config.txt:** SPI on (for the HAT), UART on, KMS disabled in favour of
  `vc4-fkms-v3d`, fixed 1080p HDMI, no splash. `cmdline.txt` adds `quiet
  nosplash` and a forced HDMI mode.
- **Bluetooth:** stock bluetoothd, enabled. See [ble.md](ble.md).

## Other things on the Pi, not copied

- `~/tmp/`: ffmpeg encoder trials on a backup-camera clip (`backup.webm`):
  AV1 (svt-av1 presets -2..13), x265, x264 and `h264_v4l2m2m` at 200–400 kbps,
  360p/320p/288p/280p/240p. 223 MB, mostly the `.y4m` intermediate. Also the
  `bumble-bench` venv.
- `~/uptime.log`: boot/uptime log, Dec 22–23 2025.
- Generated packet dumps in the old checkout: `tmp_packets/` (6700 files),
  `camera_packets/` (532), `output.h264`.
- Remotes on the old checkout: `origin` = waveshareteam/High-Pricision_AD_HAT,
  `zhuker` = git@github.com:zhuker/r22b.git. As of the last fetch on the Pi, `zhuker/master` is
  at `484bd69 probe mtu size`, so the six commits after it were probably never pushed.
