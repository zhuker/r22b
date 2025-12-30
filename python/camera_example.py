import fractions
import av
import signal
import sys

from h264_stream_sources import H264CameraStreamSource

# Flag to handle clean exit on Ctrl+C
keep_running = True

def signal_handler(sig, frame):
    global keep_running
    print("\nStopping capture...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

WIDTH=640
HEIGHT=360
FRAMERATE=30

def record_camera_low_latency():
    input_device = '/dev/video0'
    output_filename = 'output.h264'
    
    # 1. Open Input (USB Camera)
    options_cam = {
        'framerate': str(FRAMERATE),
        'video_size': f'{WIDTH}x{HEIGHT}',
    }
    # Note: 'format' must be v4l2 for Linux webcams
    in_container = av.open(input_device, format='v4l2', options=options_cam)
    in_stream = in_container.streams.video[0]

    # 2. Create codec context directly (no output container needed)
    codec = av.CodecContext.create('libx264', 'w')
    codec.width = WIDTH
    codec.height = HEIGHT
    codec.pix_fmt = 'yuv420p'
    codec.time_base = fractions.Fraction(1, FRAMERATE)
    codec.framerate = FRAMERATE
    codec.max_b_frames = 0  # No B-frames for low latency
    codec.gop_size = 600  # GOP size of 600 frames
    codec.bit_rate = 200000  # 300 kbps
    
    # --- LOW LATENCY SETTINGS ---
    # 'ultrafast': Minimize CPU usage (sacrifice compression slightly)
    # 'zerolatency': Optimize for streaming/real-time (removes buffering)
    codec.options = {
        # 'preset': 'ultrafast',
        'tune': 'zerolatency'
    }
    codec.open()

    # Open output file for writing raw H.264 data
    output_file = open(output_filename, 'wb')

    print(f"Capturing from {input_device}, writing to {output_filename}")
    print("Press Ctrl+C to stop.")

    try:
        for frame in in_container.decode(in_stream):
            if not keep_running:
                break
            frame.pict_type = av.video.frame.PictureType.NONE  # Let encoder decide frame type'

            # Encode frame and access packet data directly
            for packet in codec.encode(frame):
                # Access raw packet data using bytes() - Packet inherits Buffer protocol
                packet_data = bytes(packet)
                
                # Write to file
                output_file.write(packet_data)
                
                # Hexdump first 16 bytes
                hex_str = ' '.join(f'{b:02x}' for b in packet_data[:16])
                print(f"Packet: {len(packet_data)} bytes, keyframe={packet.is_keyframe}, pts={packet.pts}, dts={packet.dts}")
                print(f"  First 16 bytes: {hex_str}")
                
                # Now you can:
                # - Send over BLE: h264_packetizer.packetize_frame(packet_data)
                # - Send over network
                # - Write to file manually: file.write(packet_data)
                # - Process/analyze the raw H.264 data

    # except av.AVError as e:
    #     print(f"Error: {e}")
    finally:
        # 4. Flush the Encoder
        for packet in codec.encode():
            packet_data = bytes(packet)
            output_file.write(packet_data)
            hex_str = ' '.join(f'{b:02x}' for b in packet_data[:16])
            print(f"Flushed packet: {len(packet_data)} bytes")
            print(f"  First 16 bytes: {hex_str}")

        # Close files
        output_file.close()
        in_container.close()
        print("Capture finished.")

def main0():
    h264_source = H264CameraStreamSource(device='/dev/video0', width=640, height=360, framerate=30, bitrate=200000)
    h264_source.reload_frames()
    while keep_running:
        for i, packet_data in enumerate(h264_source.get_next_frame_packets()):
            print(f"Got {len(packet_data)} byte packet for frame {h264_source.current_frame}")
            with open("camera_packets/frame_{:04d}_{:03d}.bin".format(h264_source.current_frame, i), "wb") as f:
                f.write(packet_data)
            hex_str = ' '.join(f'{b:02x}' for b in packet_data[:16])
            print(f"  First 16 bytes: {hex_str}")

if __name__ == '__main__':
    main0()
    # record_camera_low_latency()