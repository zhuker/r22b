# r22b system setup

State of the Pi as read on 2026-09-14 (read-only). The customized config files
are copied under `system/`, mirroring their paths on the Pi.

## Hardware

- Raspberry Pi 4 Model B Rev 1.5, 8 GB, 64 GB SD card (6.5 GB used)
- Waveshare High-Precision AD HAT (ADS1263, SPI),
  [Amazon B09M7FLFB3](https://www.amazon.com/dp/B09M7FLFB3). Uses SPI
  GPIO 10/9/11 (pins 19/21/23), CS GPIO22 (pin 15), DRDY GPIO17 (pin 11),
  RST GPIO18 (pin 12), per Waveshare's wiki (the `r22b/adc/` driver is the
  ground truth). It sits on top of
  the 40-pin header.
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

## Access

From the Mac, SSH first, serial console when the network is down (bad
`systemd-networkd` config, boot hang, no Ethernet in the car).

- **SSH:** `ssh r22b` (the `r22b` host entry in `~/.ssh/config`; static IP
  192.168.1.50).
- **Serial console:** a tmux session `pi` running `tio`, logged to
  `~/pi-console.log`. Start it if `tmux has-session -t pi` fails:
  ```
  tmux new -d -s pi 'tio -b 115200 --log --log-file ~/pi-console.log /dev/cu.usbserial-0001'
  ```
  Read with `tmux capture-pane -pt pi` (add `-S -200` for scrollback), type with
  `tmux send-keys -t pi -l 'command'; tmux send-keys -t pi Enter`. A human
  can `tmux attach -t pi` (detach `Ctrl-b d`, quit tio `Ctrl-t q`).
- **Logging in on serial:** the username and password are lines 1 and 2 of
  `~/secrets/r22b.txt`. Send them from the file so the password never appears
  in a command line or transcript:
  ```
  tmux send-keys -t pi -l "$(sed -n 1p ~/secrets/r22b.txt)"; tmux send-keys -t pi Enter
  sleep 2
  tmux send-keys -t pi -l "$(sed -n 2p ~/secrets/r22b.txt)"; tmux send-keys -t pi Enter
  ```
  The login stays open until `exit` or a reboot. Check the pane first: if it
  already shows `zhukov@r22b:~$`, skip the login.
- **Serial tips:** it's 115200 baud, so keep commands short and
  non-interactive (`| head`, `--no-pager`, `SYSTEMD_PAGER=`). Wait a second or
  two after sending before capturing. If the pane shows `Disconnected`, the
  cable was unplugged; tio reconnects on its own when it comes back.

## Serial console

Set up 2026-09-14 with an Adafruit 954 cable (CP2102, shows up as
`/dev/cu.usbserial-0001` on the Mac). The AD HAT doesn't use the UART, so the
two coexist.

- **Wiring:** black (GND) → pin 6, green (cable TX) → pin 10 GPIO15 RXD,
  white (cable RX) → pin 8 GPIO14 TXD. Red (5 V) unconnected; the Pi has its
  own supply.
- **Reaching the pins:** the HAT covers the header. Use a 2×20 extra-long
  stacking header, or a 40-pin splitter/ribbon for the car (holds up better to
  vibration).
- **Pi config:** `enable_uart=1` in `config.txt`, and `console=serial0,115200`
  before `console=tty1` in `cmdline.txt` (this also starts
  `serial-getty@ttyS0`). The console is on
  the mini-UART (Bluetooth has the PL011). Don't add `dtoverlay=disable-bt`,
  since the BLE services need it. Optional: `BOOT_UART=1` via
  `rpi-eeprom-config --edit` for bootloader output.
- **Mac:** `brew install tio`; the CP2102 needs no driver. See
  [Access](#access) for the tmux session.
- **In the car:** keep the laptop on battery, or use a USB isolator, to avoid
  a ground loop through the laptop charger.

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
