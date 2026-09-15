"""
H.264 RTP Packetization Module

Implements H.264 packetization logic for RTP-like delivery including:
- NAL unit parsing
- Single NAL unit packets
- FU-A fragmentation for large NAL units
- STAP-A aggregation for small NAL units
"""

from typing import List, Iterator
from struct import pack

# H.264 Constants
NAL_TYPE_FU_A = 28
NAL_TYPE_STAP_A = 24
NAL_HEADER_SIZE = 1
FU_A_HEADER_SIZE = 2
STAP_A_HEADER_SIZE = 1
NAL_LENGTH_SIZE = 2
DEFAULT_MAX_PACKET_SIZE = 500


def find_nal_units(buf: bytes) -> Iterator[bytes]:
    """
    Find NAL units in H.264 bitstream.
    
    Args:
        buf: Raw H.264 bitstream with start codes
        
    Yields:
        Individual NAL unit bytes (without start codes)
    """
    i = 0
    while True:
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            return
        i += 3
        nal_start = i
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            yield buf[nal_start:len(buf)]
            return
        elif buf[i - 1] == 0:
            yield buf[nal_start:i - 1]
        else:
            yield buf[nal_start:i]


def packetize_nal(nal_unit: bytes, max_size: int = DEFAULT_MAX_PACKET_SIZE) -> List[bytes]:
    """
    Packetize single NAL unit.
    
    If NAL fits in max_size, returns it as-is.
    Otherwise fragments it using FU-A (Fragmentation Unit type A).
    
    Args:
        nal_unit: Single NAL unit bytes
        max_size: Maximum packet size in bytes
        
    Returns:
        List of packets (either single packet or FU-A fragments)
    """
    packets = []
    nal_size = len(nal_unit)
    if nal_size == 0:
        return packets
    
    nal_type = nal_unit[0] & 0x1F
    f_nri = nal_unit[0] & 0xE0
    
    if nal_size <= max_size:
        # Single NAL unit packet
        packets.append(nal_unit)
    else:
        # Fragmentation Unit - A (FU-A)
        fu_indicator = f_nri | NAL_TYPE_FU_A
        payload_size = max_size - FU_A_HEADER_SIZE
        nal_payload = nal_unit[NAL_HEADER_SIZE:]
        num_fragments = (len(nal_payload) + payload_size - 1) // payload_size
        
        for i in range(num_fragments):
            start = i * payload_size
            end = min(start + payload_size, len(nal_payload))
            
            # FU header: S(tart)|E(nd)|R|Type
            fu_header = nal_type
            if i == 0:
                fu_header |= 0x80  # Start bit
            if i == num_fragments - 1:
                fu_header |= 0x40  # End bit
                
            packet = bytes([fu_indicator, fu_header]) + nal_payload[start:end]
            packets.append(packet)
    
    return packets


def create_stap_a_packet(nal_units: List[bytes], max_size: int = DEFAULT_MAX_PACKET_SIZE) -> bytes:
    """
    Create STAP-A (Single Time Aggregation Packet type A) from multiple NAL units.
    
    Used to bundle multiple small NAL units into a single packet.
    
    Args:
        nal_units: List of NAL unit bytes to aggregate
        max_size: Maximum packet size (for validation)
        
    Returns:
        STAP-A packet bytes
    """
    if not nal_units:
        return b''
    
    # Use F and NRI from first NAL
    f_nri = nal_units[0][0] & 0xE0
    stap_header = f_nri | NAL_TYPE_STAP_A
    
    packet = bytes([stap_header])
    for nal_unit in nal_units:
        nal_size = len(nal_unit)
        # Each NAL prefixed with 2-byte size
        packet += pack('>H', nal_size)
        packet += nal_unit
    
    return packet


def packetize_frame(frame_data: bytes, max_size: int = DEFAULT_MAX_PACKET_SIZE, 
                    use_aggregation: bool = True) -> List[bytes]:
    """
    Packetize complete H.264 frame.
    
    Implements smart aggregation and fragmentation:
    - Small NAL units are aggregated into STAP-A packets
    - Large NAL units are fragmented into FU-A packets
    - Flushes aggregation buffer before fragmenting large NALs
    
    Args:
        frame_data: Complete H.264 frame data with start codes
        max_size: Maximum packet size in bytes
        use_aggregation: Whether to use STAP-A aggregation
        
    Returns:
        List of packetized payloads ready for transmission
    """
    all_packets = []
    nal_units = list(find_nal_units(frame_data))
    
    if not use_aggregation:
        # Simple mode: just packetize each NAL individually
        for nal_unit in nal_units:
            all_packets.extend(packetize_nal(nal_unit, max_size))
        return all_packets
    
    # Aggregation mode
    aggregation_buffer = []
    aggregation_size = STAP_A_HEADER_SIZE
    
    for nal_unit in nal_units:
        nal_size = len(nal_unit)
        needed_size = NAL_LENGTH_SIZE + nal_size
        needs_fragmentation = nal_size > max_size
        
        if needs_fragmentation:
            # Flush aggregation buffer before fragmenting
            if aggregation_buffer:
                if len(aggregation_buffer) > 1:
                    stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                    all_packets.append(stap_packet)
                elif len(aggregation_buffer) == 1:
                    all_packets.append(aggregation_buffer[0])
                aggregation_buffer = []
                aggregation_size = STAP_A_HEADER_SIZE
            
            # Fragment large NAL
            packets = packetize_nal(nal_unit, max_size)
            all_packets.extend(packets)
            
        elif aggregation_size + needed_size <= max_size:
            # Add to aggregation buffer
            aggregation_buffer.append(nal_unit)
            aggregation_size += needed_size
        else:
            # Flush buffer and start new one
            if len(aggregation_buffer) > 1:
                stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                all_packets.append(stap_packet)
            elif len(aggregation_buffer) == 1:
                all_packets.append(aggregation_buffer[0])
            
            aggregation_buffer = [nal_unit]
            aggregation_size = STAP_A_HEADER_SIZE + needed_size
    
    # Flush remaining buffer
    if aggregation_buffer:
        if len(aggregation_buffer) > 1:
            stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
            all_packets.append(stap_packet)
        elif len(aggregation_buffer) == 1:
            all_packets.append(aggregation_buffer[0])
    
    return all_packets


def read_h264_frame(filepath: str) -> bytes:
    """
    Read H.264 frame from file.
    
    Args:
        filepath: Path to H.264 frame file
        
    Returns:
        Frame data bytes
    """
    with open(filepath, 'rb') as f:
        return f.read()
