"""
RealDash CAN protocol framing (the 0x44 frame type).

Each frame is 16 bytes: 44 33 22 11 header, 4-byte little-endian CAN id,
8 data bytes. The matching RealDash channel definition is
data/realdash/realdash_5.xml. Split out of bumble_service.py; logic unchanged.
"""

CAN_ID_DIFF_TEMP = 3200  # 0xC80
CAN_ID_TPMS = {'FL': 3201, 'FR': 3202, 'RL': 3203, 'RR': 3204}  # 0xC81..0xC84


def build_realdash_frame(can_id, value):
    """Packs data into RealDash CAN Protocol."""
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    temp_int = int(value * 10)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    frame.extend(bytearray([0,0,0,0,0,0]))
    return bytes(frame)


def build_tpms_frame(can_id, pressure_bar, temp_c, leaking, battery_low):
    """Builds TPMS CAN frame."""
    frame = bytearray([0x44, 0x33, 0x22, 0x11])
    frame.extend(can_id.to_bytes(4, byteorder='little'))
    pressure_int = int(pressure_bar * 100)
    frame.extend(pressure_int.to_bytes(2, byteorder='little', signed=False))
    temp_int = int(temp_c * 10)
    frame.extend(temp_int.to_bytes(2, byteorder='little', signed=True))
    frame.append(leaking & 0xFF)
    frame.append(battery_low & 0xFF)
    frame.extend(bytearray([0, 0]))
    return bytes(frame)
