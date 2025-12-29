"""
H.264 Frame Packetizer for RTP delivery

Reads H.264 frames from files and packetizes them into RTP-ready packets.
Based on aiortc's H.264 packetization implementation.
"""

import os
import glob
from typing import List, Iterator
from struct import pack


# Constants from RTP H.264 payload specification
PACKET_MAX = 500  # Maximum packet size in bytes
NAL_TYPE_FU_A = 28  # Fragmentation Unit A
NAL_TYPE_STAP_A = 24  # Single-Time Aggregation Packet A
NAL_HEADER_SIZE = 1
FU_A_HEADER_SIZE = 2
STAP_A_HEADER_SIZE = 1
NAL_LENGTH_SIZE = 2  # 2 bytes for NAL unit length in STAP-A

# NAL unit types that are typically small and can be aggregated
AGGREGATABLE_NAL_TYPES = {
    6,  # SEI (Supplemental Enhancement Information)
    7,  # SPS (Sequence Parameter Set)
    8,  # PPS (Picture Parameter Set)
    9,  # Access Unit Delimiter
}


def find_nal_units(buf: bytes) -> Iterator[bytes]:
    """
    Find NAL units in a H.264 bitstream.
    
    NAL Units start with the 3-byte start code 0x000001 or
    the 4-byte start code 0x00000001.
    
    Args:
        buf: H.264 bitstream buffer
        
    Yields:
        Individual NAL units (without start codes)
    """
    i = 0
    while True:
        # Find the start of the NAL unit
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            return
        
        # Jump past the start code
        i += 3
        nal_start = i
        
        # Find the end of the NAL unit (end of buffer OR next start code)
        i = buf.find(b"\x00\x00\x01", i)
        if i == -1:
            yield buf[nal_start:len(buf)]
            return
        elif buf[i - 1] == 0:
            # 4-byte start code case, jump back one byte
            yield buf[nal_start:i - 1]
        else:
            yield buf[nal_start:i]


def packetize_nal(nal_unit: bytes, max_size: int = PACKET_MAX) -> List[bytes]:
    """
    Packetize a single NAL unit into RTP payload packets.
    
    Small NAL units are sent as single packets.
    Large NAL units are fragmented using FU-A packets.
    
    Args:
        nal_unit: Single NAL unit (without start code)
        max_size: Maximum packet size in bytes
        
    Returns:
        List of RTP payload packets
    """
    packets = []
    nal_size = len(nal_unit)
    
    if nal_size == 0:
        return packets
    
    # Extract NAL header info
    nal_type = nal_unit[0] & 0x1F
    f_nri = nal_unit[0] & 0xE0  # F (1 bit) + NRI (2 bits)
    
    # Single NAL Unit Packet (fits in one packet)
    if nal_size <= max_size:
        packets.append(nal_unit)
    else:
        # Fragment NAL unit into FU-A packets
        # FU-A format:
        # +---------------+
        # |0|1|2|3|4|5|6|7|
        # +-+-+-+-+-+-+-+-+
        # |F|NRI|  Type   |  FU indicator (Type = 28 for FU-A)
        # +---------------+
        # |S|E|R|  Type   |  FU header
        # +---------------+
        # |   Payload     |
        # +---------------+
        
        fu_indicator = f_nri | NAL_TYPE_FU_A
        
        # Calculate payload size per packet
        payload_size = max_size - FU_A_HEADER_SIZE
        nal_payload = nal_unit[NAL_HEADER_SIZE:]  # Skip original NAL header
        
        num_fragments = (len(nal_payload) + payload_size - 1) // payload_size
        
        for i in range(num_fragments):
            start = i * payload_size
            end = min(start + payload_size, len(nal_payload))
            
            # FU header: S (start) | E (end) | R (reserved=0) | Type (5 bits)
            fu_header = nal_type
            if i == 0:
                fu_header |= 0x80  # Set Start bit
            if i == num_fragments - 1:
                fu_header |= 0x40  # Set End bit
            
            # Build FU-A packet
            packet = bytes([fu_indicator, fu_header]) + nal_payload[start:end]
            packets.append(packet)
    
    return packets


def create_stap_a_packet(nal_units: List[bytes], max_size: int = PACKET_MAX) -> bytes:
    """
    Create a STAP-A packet from multiple small NAL units.
    
    STAP-A format (RFC 6184):
    +---------------+
    |0|1|2|3|4|5|6|7|
    +-+-+-+-+-+-+-+-+
    |F|NRI|  Type   |  STAP-A NAL header (Type = 24)
    +---------------+
    |   NAL Size    |  2 bytes - size of first NAL unit
    +---------------+
    |   NAL Unit    |
    +---------------+
    |   NAL Size    |  2 bytes - size of next NAL unit
    +---------------+
    |   NAL Unit    |
    +---------------+
    
    Args:
        nal_units: List of NAL units to aggregate
        max_size: Maximum packet size
        
    Returns:
        STAP-A packet bytes
    """
    if not nal_units:
        return b''
    
    # Use the NRI from the first NAL unit (they should all have similar priority)
    f_nri = nal_units[0][0] & 0xE0
    stap_header = f_nri | NAL_TYPE_STAP_A
    
    packet = bytes([stap_header])
    
    for nal_unit in nal_units:
        # Add 2-byte length prefix (network byte order = big endian)
        nal_size = len(nal_unit)
        packet += pack('>H', nal_size)  # >H = big-endian unsigned short
        packet += nal_unit
    
    return packet


def packetize_frame(frame_data: bytes, max_size: int = PACKET_MAX, use_aggregation: bool = True) -> List[bytes]:
    """
    Packetize a complete H.264 frame into RTP payload packets.
    
    Small NAL units (SPS, PPS, SEI) can be aggregated into STAP-A packets
    to reduce overhead and improve efficiency.
    
    Args:
        frame_data: Complete H.264 frame with start codes
        max_size: Maximum packet size in bytes
        use_aggregation: Whether to use STAP-A aggregation for small NAL units
        
    Returns:
        List of RTP payload packets ready for transmission
    """
    all_packets = []
    
    # Split frame into NAL units
    nal_units = list(find_nal_units(frame_data))
    
    if not use_aggregation:
        # Simple mode: packetize each NAL unit individually
        for nal_unit in nal_units:
            packets = packetize_nal(nal_unit, max_size)
            all_packets.extend(packets)
    else:
        # Aggregation mode: group small NAL units together
        # All NAL units in a frame belong to the same access unit (timestamp)
        # so they can all be aggregated if they fit
        aggregation_buffer = []
        aggregation_size = STAP_A_HEADER_SIZE
        
        for nal_unit in nal_units:
            nal_type = nal_unit[0] & 0x1F
            nal_size = len(nal_unit)
            
            # Calculate size if we add this NAL to the aggregation
            # Each NAL in STAP-A needs: 2 bytes (length) + NAL data
            needed_size = NAL_LENGTH_SIZE + nal_size
            
            # Check if NAL would need fragmentation if sent alone
            needs_fragmentation = nal_size > max_size
            
            if needs_fragmentation:
                # This NAL is too large to aggregate, must fragment it
                # First, flush any pending aggregation
                if aggregation_buffer:
                    if len(aggregation_buffer) > 1:
                        stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                        all_packets.append(stap_packet)
                    elif len(aggregation_buffer) == 1:
                        all_packets.append(aggregation_buffer[0])
                    aggregation_buffer = []
                    aggregation_size = STAP_A_HEADER_SIZE
                
                # Fragment this large NAL unit
                packets = packetize_nal(nal_unit, max_size)
                all_packets.extend(packets)
            elif aggregation_size + needed_size <= max_size:
                # This NAL fits in the current aggregation buffer
                aggregation_buffer.append(nal_unit)
                aggregation_size += needed_size
            else:
                # Current buffer is full, flush it and start new aggregation
                if len(aggregation_buffer) > 1:
                    stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                    all_packets.append(stap_packet)
                elif len(aggregation_buffer) == 1:
                    all_packets.append(aggregation_buffer[0])
                
                # Start new aggregation with current NAL
                aggregation_buffer = [nal_unit]
                aggregation_size = STAP_A_HEADER_SIZE + needed_size
        
        # Flush any remaining aggregated NAL units
        if aggregation_buffer:
            if len(aggregation_buffer) > 1:
                stap_packet = create_stap_a_packet(aggregation_buffer, max_size)
                all_packets.append(stap_packet)
            elif len(aggregation_buffer) == 1:
                all_packets.append(aggregation_buffer[0])
    
    return all_packets


def read_h264_frame(filepath: str) -> bytes:
    """
    Read a H.264 frame from a file.
    
    Args:
        filepath: Path to the H.264 frame file
        
    Returns:
        Raw H.264 frame data
    """
    with open(filepath, 'rb') as f:
        return f.read()


def save_packets(packets: List[bytes], frame_name: str, output_dir: str = "tmp_packets"):
    """
    Save packets to individual files.
    
    Args:
        packets: List of packet bytes to save
        frame_name: Name of the source frame (e.g., "frame-0001.h264")
        output_dir: Directory to save packet files
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract frame number from filename (e.g., "frame-0001.h264" -> "0001")
    frame_num = frame_name.replace("frame-", "").replace(".h264", "")
    
    for i, packet in enumerate(packets, start=1):
        packet_filename = f"frame_{frame_num}_packet_{i:03d}.bin"
        packet_path = os.path.join(output_dir, packet_filename)
        
        with open(packet_path, 'wb') as f:
            f.write(packet)


def process_h264_frames(frames_dir: str = "h264SampleFrames", max_packet_size: int = PACKET_MAX):
    """
    Process all H.264 frames in a directory and packetize them.
    
    Args:
        frames_dir: Directory containing H.264 frame files
        max_packet_size: Maximum size for each packet in bytes
    """
    # Get all frame files sorted
    frame_files = sorted(glob.glob(os.path.join(frames_dir, "frame-*.h264")))
    
    if not frame_files:
        print(f"No H.264 frame files found in {frames_dir}")
        return
    
    print(f"Found {len(frame_files)} H.264 frame files")
    print(f"Maximum packet size: {max_packet_size} bytes\n")
    
    total_packets = 0
    
    for frame_file in frame_files:
        # Read frame data
        frame_data = read_h264_frame(frame_file)
        frame_size = len(frame_data)
        
        # Packetize the frame
        packets = packetize_frame(frame_data, max_packet_size)
        
        # Statistics
        filename = os.path.basename(frame_file)
        num_packets = len(packets)
        total_packets += num_packets
        
        # Save packets to files
        save_packets(packets, filename)
        
        print(f"{filename}:")
        print(f"  Frame size: {frame_size:,} bytes")
        print(f"  NAL units: {len(list(find_nal_units(frame_data)))}")
        print(f"  Packets generated: {num_packets}")
        
        if num_packets > 0:
            packet_sizes = [len(p) for p in packets]
            print(f"  Packet sizes: min={min(packet_sizes)}, max={max(packet_sizes)}, avg={sum(packet_sizes)//num_packets}")
        
        for i, packet in enumerate(packets):
            nal_type = packet[0] & 0x1F
            if nal_type == NAL_TYPE_FU_A:
                fu_header = packet[1]
                is_start = bool(fu_header & 0x80)
                is_end = bool(fu_header & 0x40)
                original_type = fu_header & 0x1F
                print(f"    Packet {i+1}: FU-A fragment (type={original_type}, start={is_start}, end={is_end}, size={len(packet)})")
            elif nal_type == NAL_TYPE_STAP_A:
                # Parse STAP-A to show aggregated NAL units
                aggregated_types = []
                pos = 1  # Skip STAP-A header
                while pos + 2 <= len(packet):
                    nal_size = (packet[pos] << 8) | packet[pos + 1]
                    pos += 2
                    if pos + nal_size <= len(packet):
                        aggregated_nal_type = packet[pos] & 0x1F
                        aggregated_types.append(aggregated_nal_type)
                        pos += nal_size
                    else:
                        break
                print(f"    Packet {i+1}: STAP-A aggregation (types={aggregated_types}, count={len(aggregated_types)}, size={len(packet)})")
            else:
                print(f"    Packet {i+1}: Single NAL (type={nal_type}, size={len(packet)})")
        
        print()
    
    print(f"Total packets generated: {total_packets}")
    print(f"Average packets per frame: {total_packets / len(frame_files):.1f}")


if __name__ == "__main__":
    # Process frames from the h264SampleFrames directory
    process_h264_frames()
