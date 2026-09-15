"""
Offline tests: run on a laptop, no Pi hardware, HAT, camera or radio needed.

    .venv/bin/python -m pytest tests
"""

import asyncio
import glob
import os
import struct

import pytest

from r22b import diff_temp, realdash, tpms
from r22b.video import depacketize, packetizer

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAMPLE_FRAMES = sorted(glob.glob(os.path.join(REPO, "data/samples/h264/frame-*.h264")))


# --- H.264 packetizer <-> depacketizer --------------------------------------

def test_sample_frames_present():
    assert len(SAMPLE_FRAMES) >= 50


@pytest.mark.parametrize("max_size", [100, 244, 500])
def test_packetize_roundtrip(tmp_path, max_size):
    expected_nals = []
    for n, path in enumerate(SAMPLE_FRAMES, start=1):
        frame = packetizer.read_h264_frame(path)
        expected_nals.extend(packetizer.find_nal_units(frame))
        packets = packetizer.packetize_frame(frame, max_size)
        for i, pkt in enumerate(packets, start=1):
            assert len(pkt) <= max_size
            (tmp_path / f"frame_{n:04d}_packet_{i:03d}.bin").write_bytes(pkt)

    assert list(depacketize.depacketize(str(tmp_path))) == expected_nals


def test_first_sample_frame_is_idr_with_sps_pps():
    nal_types = [nal[0] & 0x1F for nal in packetizer.find_nal_units(packetizer.read_h264_frame(SAMPLE_FRAMES[0]))]
    assert 7 in nal_types and 8 in nal_types and 5 in nal_types


# --- TPMS -------------------------------------------------------------------

def test_decode_tpms():
    raw = bytes(6) + struct.pack("<I", 223200) + struct.pack("<I", 1850) + bytes([95, 0])
    pressure_bar, temp_c, battery_low, leaking = tpms.decode_tpms(raw)
    assert pressure_bar == pytest.approx(2.232)
    assert temp_c == pytest.approx(18.5)
    assert (battery_low, leaking) == (0, 0)


def test_decode_tpms_short_payload_is_zeroed():
    assert tpms.decode_tpms(b"\x00" * 4) == (0.0, 0.0, 0, 0)


# --- RealDash CAN frames ----------------------------------------------------

def test_realdash_frame_layout():
    frame = realdash.build_realdash_frame(realdash.CAN_ID_DIFF_TEMP, -12.3)
    assert len(frame) == 16
    assert frame[:4] == bytes([0x44, 0x33, 0x22, 0x11])
    assert struct.unpack("<I", frame[4:8])[0] == 0xC80
    assert struct.unpack("<h", frame[8:10])[0] == -123


def test_tpms_frame_layout():
    frame = realdash.build_tpms_frame(realdash.CAN_ID_TPMS['RR'], 2.23, 18.5, 1, 1)
    assert len(frame) == 16
    assert struct.unpack("<I", frame[4:8])[0] == 0xC84
    assert struct.unpack("<HhBB", frame[8:14]) == (223, 185, 1, 1)


# --- Diff temp NTC table ----------------------------------------------------

def test_calibration_points_and_interpolation():
    assert diff_temp.get_temp_from_resistance(9319) == 0
    assert diff_temp.get_temp_from_resistance(468) == 70
    assert 20 < diff_temp.get_temp_from_resistance(3000) < 30
    assert diff_temp.get_temp_from_resistance(10**6) == -20   # open / very cold clamps
    assert diff_temp.get_temp_from_resistance(1) == 150


# --- Bumble GATT service over a virtual link --------------------------------

class FakeSensorReader:
    def read_temp(self):
        return 42.5


def test_gatt_service_streams_realdash_frames():
    """Central connects to the real BumbleGATTServer over Bumble's in-process link."""
    from bumble.controller import Controller
    from bumble.device import Device, Peer
    from bumble.hci import Address
    from bumble.host import Host
    from bumble.link import LocalLink
    from bumble.transport.common import AsyncPipeSink

    from r22b import ble_service
    from r22b.video.sources import H264FileStreamSource

    async def run():
        link = LocalLink()
        devices = []
        for i, addr in enumerate(["F0:F0:F0:F0:F0:F0", "F1:F1:F1:F1:F1:F1"]):
            controller = Controller(f"C{i}", link=link, public_address=addr)
            devices.append(Device(address=Address(addr), host=Host(controller, AsyncPipeSink(controller))))
        central, peripheral = devices

        tpms_data = tpms.TPMSData()
        tpms_data.update('FR', 2.25, 19.0, 0, 0)
        server = ble_service.BumbleGATTServer(
            peripheral, FakeSensorReader(), tpms_data,
            H264FileStreamSource(frames_dir=os.path.join(REPO, "data/samples/h264")))

        for d in devices:
            await d.power_on()
        connected = asyncio.get_running_loop().create_future()
        peripheral.once(peripheral.EVENT_CONNECTION, connected.set_result)
        await peripheral.start_advertising(advertising_interval_min=1.0)
        connection = await central.connect(peripheral.random_address)
        await connected

        peer = Peer(connection)
        await peer.discover_services()
        await peer.discover_characteristics()
        [nus_tx] = peer.get_characteristics_by_uuid(ble_service.UUID(ble_service.NUS_TX_CHAR_UUID))

        received = []
        await peer.subscribe(nus_tx, received.append)
        for _ in range(20):
            if server.nus_tx_subscribed:
                break
            await asyncio.sleep(0.01)
        assert server.nus_tx_subscribed

        await server.send_realdash_data()
        for _ in range(50):
            if len(received) >= 5:
                break
            await asyncio.sleep(0.01)

        assert len(received) == 5
        by_id = {struct.unpack("<I", bytes(f[4:8]))[0]: bytes(f) for f in received}
        assert set(by_id) == {0xC80, 0xC81, 0xC82, 0xC83, 0xC84}
        assert struct.unpack("<h", by_id[0xC80][8:10])[0] == 425
        assert struct.unpack("<H", by_id[0xC82][8:10])[0] == 225

    asyncio.run(run())
