# r22b

Raspberry Pi prototype for the 22B replica: rear diff temperature, BLE TPMS
and a backup camera, streamed over BLE to the dashboard app
([dash22b](../dash22b)) in RealDash CAN format. Plus early SSM (K-line) ECU
polling work.

Extracted and reorganized on 2026-09-14 from the untidy checkout on the Pi
(`r22b:~/git/High-Pricision_AD_HAT`, which began as a clone of Waveshare's AD
HAT demo). The full history is kept: Waveshare's commits, Alex's Dec 2025
commits, then a snapshot commit of the uncommitted work before the reorg.

Design context lives in the 22b build docs: `questions/dash22b-rpi-gateway.html`
and `questions/rpi-rke-integration.html`.

## Status (as of the last work on the Pi, Jan 3 2026)

| Piece | State |
|---|---|
| Rear diff temp via ADS1263 + AEM NTC | working |
| BLE TPMS sensor decode (Bleak) | working with bluetoothd; not with the Bumble service |
| BLE GATT peripheral, RealDash frames | working (bluezero, then Bumble) |
| H.264 over BLE notifications | "camera frames streamed ok" (Bumble, USB cam) |
| SSM ECU polling (PiMonitor, Python 3 port) | reads ROM ID, supported params, polls 7 params |
| Door lock / RKE | not started |

Known issues are listed at the end of [docs/ble.md](docs/ble.md).

## Layout

```
r22b/               the Pi service package
  ble_service.py      Bumble GATT server (python -m r22b.ble_service)
  diff_temp.py        ADS1263 + AEM 30-2012 NTC table
  tpms.py             TPMS advertisement decode + Bleak scanner
  realdash.py         RealDash CAN 0x44 frame builders
  adc/                Waveshare ADS1263 driver (MIT)
  video/              H.264 packetizer, depacketizer, file/camera sources
experiments/        one-off probes kept as they were (ADC, TPMS scan, SSM init,
                    camera capture); legacy/ has the bluezero-era services
tools/              H.264 packetize-and-dump script
ssm/                ECU work: vendored PiMonitor (+ Python 3 port, RealDash
                    mapping), supported-parameter lists for this ECU
data/               TPMS scan capture, 50 sample H.264 frames, RealDash XML
system/             customized Pi config files (boot, networkd, journald)
docs/               ble.md, pi-system.md
tests/              offline tests, no hardware needed
```

## Laptop: run the tests

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest
```

The GATT test runs the real `BumbleGATTServer` against a Bumble central over an
in-process virtual link.

## Pi: run the service

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
sudo systemctl stop bluetooth && sudo hciconfig hci0 down
sudo .venv/bin/python -m r22b.ble_service
```

Experiments run from the repo root with `PYTHONPATH=.`, e.g.
`PYTHONPATH=. python3 experiments/adc_probe.py`. PiMonitor scripts run from
`ssm/PiMonitor` with `PYTHONPATH=. python3 pimonitor/climain.py`.
