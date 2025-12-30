"""
H.264 Stream Source Implementations

Provides abstract base class and concrete implementations for H.264 video streaming:
- H264FileStreamSource: Reads pre-encoded H.264 files from disk
- H264CameraStreamSource: Captures live video from camera and encodes in real-time
"""

import glob
import fractions
import logging
from typing import List, Dict
from abc import ABC, abstractmethod

# Local imports
import h264_packetizer

# PyAV for camera capture
import av

logger = logging.getLogger(__name__)


class H264StreamSource(ABC):
    """Abstract base class for H.264 stream sources."""
    
    @abstractmethod
    def reload_frames(self):
        """Reload/reinitialize the stream source."""
        pass
    
    @abstractmethod
    def reset(self):
        """Reset stream to beginning."""
        pass
    
    @abstractmethod
    def has_frames(self) -> bool:
        """Check if frames are available."""
        pass
    
    @abstractmethod
    def get_next_frame_packets(self) -> List[bytes]:
        """Get packetized data for next frame."""
        pass
    
    @abstractmethod
    def get_stats(self) -> Dict[str, int]:
        """Get streaming statistics."""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Cleanup resources."""
        pass


class H264FileStreamSource(H264StreamSource):
    """H.264 stream source that reads from files on disk."""
    
    def __init__(self, frames_dir: str = "h264SampleFrames"):
        self.frames_dir = frames_dir
        self.frame_files: List[str] = []
        self.current_frame = 0
        self.total_packets_sent = 0
    
    def _load_frame_files(self):
        """Load available frame files from directory."""
        self.frame_files = sorted(glob.glob(f"{self.frames_dir}/frame-*.h264"))
        self.current_frame = 0
    
    def reload_frames(self):
        """Reload frame files from directory."""
        self._load_frame_files()
    
    def reset(self):
        """Reset to first frame."""
        self.current_frame = 0
    
    def has_frames(self) -> bool:
        """Check if frames are available."""
        return len(self.frame_files) > 0
    
    def get_next_frame_packets(self) -> List[bytes]:
        """Get packetized data for next frame."""
        if not self.frame_files:
            return []
        
        # Loop back to start if we've reached the end
        if self.current_frame >= len(self.frame_files):
            self.current_frame = 0
        
        frame_file = self.frame_files[self.current_frame]
        
        # Read and packetize frame
        frame_data = h264_packetizer.read_h264_frame(frame_file)
        packets = h264_packetizer.packetize_frame(frame_data)
        
        self.current_frame += 1
        self.total_packets_sent += len(packets)
        
        return packets
    
    def get_stats(self) -> Dict[str, int]:
        """Get streaming statistics."""
        return {
            'current_frame': self.current_frame,
            'total_frames': len(self.frame_files),
            'total_packets_sent': self.total_packets_sent
        }
    
    def cleanup(self):
        """Cleanup resources (no-op for file source)."""
        pass


class H264CameraStreamSource(H264StreamSource):
    """H.264 stream source that captures from camera and encodes in real-time."""
    
    def __init__(self, device: str = '/dev/video0', width: int = 640, height: int = 360, 
                 framerate: int = 30, bitrate: int = 300000):
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.bitrate = bitrate
        
        self.in_container = None
        self.in_stream = None
        self.codec = None
        self.frame_iterator = None
        
        self.current_frame = 0
        self.total_packets_sent = 0
        self.is_active = False
        
    def _initialize_camera(self):
        """Initialize camera capture and encoder."""
        try:
            # Open camera input
            options_cam = {
                'framerate': str(self.framerate),
                'video_size': f'{self.width}x{self.height}',
            }
            self.in_container = av.open(self.device, format='v4l2', options=options_cam)
            self.in_stream = self.in_container.streams.video[0]
            
            # Create codec context
            self.codec = av.CodecContext.create('libx264', 'w')
            self.codec.width = self.width
            self.codec.height = self.height
            self.codec.pix_fmt = 'yuv420p'
            self.codec.time_base = fractions.Fraction(1, self.framerate)
            self.codec.framerate = self.framerate
            self.codec.max_b_frames = 0  # No B-frames for low latency
            self.codec.gop_size = 600  # GOP size of 600 frames
            self.codec.bit_rate = self.bitrate
            
            self.codec.options = {
                #'preset': 'ultrafast',
                'tune': 'zerolatency'
            }
            self.codec.open()
            
            # Create frame iterator
            self.frame_iterator = self.in_container.decode(self.in_stream)
            self.is_active = True
            
            logger.info(f"Camera initialized: {self.device} at {self.width}x{self.height}@{self.framerate}fps")
            
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            self.is_active = False
    
    def reload_frames(self):
        """Reload/reinitialize camera (cleanup and restart)."""
        self.cleanup()
        self.current_frame = 0
        self.total_packets_sent = 0
        self._initialize_camera()
    
    def reset(self):
        """Reset frame counter (camera stream doesn't reset like files)."""
        self.current_frame = 0
    
    def has_frames(self) -> bool:
        """Check if camera is active."""
        return self.is_active
    
    def get_next_frame_packets(self) -> List[bytes]:
        """Capture next frame from camera and return packetized data."""
        if not self.is_active or not self.frame_iterator:
            return []
        
        try:
            # Get next frame from camera
            frame = next(self.frame_iterator)
            if self.current_frame == 0:
                frame.pict_type = av.video.frame.PictureType.I  # Force first frame as I-frame
            else:
                frame.pict_type = av.video.frame.PictureType.NONE  # Let encoder decide frame type'

            # Encode frame to H.264
            packets_list = []
            for packet in self.codec.encode(frame):
                packet_data = bytes(packet)
                packets_list.append(packet_data)
                hex_str = ' '.join(f'{b:02x}' for b in packet_data[:16])
                print(f"h264 from encoder: {hex_str}")
                
            # Concatenate all encoded packets into one buffer
            if packets_list:
                # access unit delimiter
                packets_list = [bytes([00, 00, 00, 0x01, 0x09, 0x10])] + packets_list
                frame_data = b''.join(packets_list)
                
                # Packetize for BLE transmission
                ble_packets = h264_packetizer.packetize_frame(frame_data)
                
                self.current_frame += 1
                self.total_packets_sent += len(ble_packets)
                
                return ble_packets
            
            return []
            
        except StopIteration:
            # Camera stream ended, try to restart
            logger.warning("Camera stream ended, attempting to reload...")
            self.reload_frames()
            return []
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return []
    
    def get_stats(self) -> Dict[str, int]:
        """Get streaming statistics."""
        return {
            'current_frame': self.current_frame,
            'total_frames': self.current_frame,  # Camera has no predefined total
            'total_packets_sent': self.total_packets_sent
        }
    
    def cleanup(self):
        """Cleanup camera and encoder resources."""
        try:
            if self.codec:
                # Flush encoder
                for packet in self.codec.encode():
                    pass  # Discard flush packets
            
            if self.in_container:
                self.in_container.close()
            
            self.frame_iterator = None
            self.is_active = False
            
            logger.info("Camera resources cleaned up")
            
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
