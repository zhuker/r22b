"""
BLE GATT Server using Bumble
Provides RealDash CAN data streaming and H.264 video streaming

Rewritten from hello.py (experiments/legacy/bluezero_service.py) using Google Bumble.
Run on the Pi as root with bluetoothd stopped, from the repo root:

    sudo systemctl stop bluetooth
    sudo hciconfig hci0 down
    sudo .venv/bin/python -m r22b.ble_service
"""

import asyncio
import logging

# Local imports
from r22b.diff_temp import SensorReader
from r22b.tpms import TPMSData, tpms_scanner_loop
from r22b.realdash import build_realdash_frame, build_tpms_frame
from r22b.video.sources import H264StreamSource, H264FileStreamSource, H264CameraStreamSource

# Bumble imports
from bumble.device import Device, Connection, AdvertisingType
from bumble.gatt import (
    Service,
    Characteristic,
    CharacteristicValue,
)
from bumble.transport import open_transport
from bumble.core import UUID, AdvertisingData
from bumble import data_types

# Bumble GATT property constants
PROPERTY_READ = Characteristic.Properties.READ
PROPERTY_WRITE = Characteristic.Properties.WRITE
PROPERTY_WRITE_WITHOUT_RESPONSE = Characteristic.Properties.WRITE_WITHOUT_RESPONSE
PROPERTY_NOTIFY = Characteristic.Properties.NOTIFY
PROPERTY_INDICATE = Characteristic.Properties.INDICATE

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# BLE UUIDs
NUS_SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_RX_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
NUS_TX_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
H264_STREAM_CHAR_UUID = '6E400012-B5A3-F393-E0A9-E50E24DCCA9E'
H264_CONTROL_CHAR_UUID = '6E400013-B5A3-F393-E0A9-E50E24DCCA9E'

# --- GATT Server Implementation ---
class BumbleGATTServer:
    def __init__(self, device: Device, sensor_reader: SensorReader, tpms_data: TPMSData,
                 h264_source: H264StreamSource):
        self.device = device
        self.sensor_reader = sensor_reader
        self.tpms_data = tpms_data
        self.h264_source = h264_source
        
        # Characteristic references
        self.nus_tx_char = None
        self.h264_stream_char = None
        
        # Subscription state
        self.nus_tx_subscribed = False
        self.h264_stream_subscribed = False
        
        # H.264 streaming state
        self.h264_streaming_active = False
        
        # Statistics
        self.counter = 0
        
        # Setup services
        self.setup_services()
        
    def setup_services(self):
        """Setup GATT services and characteristics."""
        # NUS Service
        nus_service = Service(
            NUS_SERVICE_UUID,
            [
                # TX Characteristic (Notify) - RealDash CAN data
                Characteristic(
                    NUS_TX_CHAR_UUID,
                    PROPERTY_NOTIFY,
                    CharacteristicValue(read=self._read_nus_tx),
                ),
                # RX Characteristic (Write)
                Characteristic(
                    NUS_RX_CHAR_UUID,
                    PROPERTY_WRITE | PROPERTY_WRITE_WITHOUT_RESPONSE,
                    CharacteristicValue(write=self._write_nus_rx),
                ),
                # H.264 Stream Characteristic (Notify)
                Characteristic(
                    H264_STREAM_CHAR_UUID,
                    PROPERTY_NOTIFY,
                    CharacteristicValue(read=self._read_h264_stream),
                ),
                # H.264 Control Characteristic (Write)
                Characteristic(
                    H264_CONTROL_CHAR_UUID,
                    PROPERTY_WRITE | PROPERTY_WRITE_WITHOUT_RESPONSE,
                    CharacteristicValue(write=self._write_h264_control),
                ),
            ]
        )
        
        self.device.add_service(nus_service)
        
        # Store characteristic references for notifications
        self.nus_tx_char = nus_service.characteristics[0]
        self.h264_stream_char = nus_service.characteristics[2]
        
        # Subscribe to characteristic subscription events
        self.nus_tx_char.on('subscription', self._on_nus_tx_subscription)
        self.h264_stream_char.on('subscription', self._on_h264_subscription)
        
    def _read_nus_tx(self, connection):
        """Read handler for NUS TX characteristic."""
        return b''
    
    def _write_nus_rx(self, connection, value):
        """Write handler for NUS RX characteristic."""
        pass
    
    def _read_h264_stream(self, connection):
        """Read handler for H.264 stream characteristic."""
        return b''
    
    def _write_h264_control(self, connection, value):
        """Write handler for H.264 control characteristic."""
        cmd = value.decode('utf-8', errors='ignore').strip()
        logger.info(f"H.264 Control: {cmd}")
        
        if cmd == "START":
            self.h264_streaming_active = True
            self.h264_source.reload_frames()
            logger.info(f"H.264: Starting stream)")
        elif cmd == "STOP":
            self.h264_streaming_active = False
            logger.info("H.264: Stopping stream")
        elif cmd == "RESET":
            self.h264_source.reset()
            logger.info("H.264: Reset to frame 0")
    
    def _on_nus_tx_subscription(self, connection, notify_enabled, indicate_enabled):
        """Handle subscription changes for NUS TX."""
        self.nus_tx_subscribed = notify_enabled
        if notify_enabled:
            logger.info("RealDash Connected! Streaming Data...")
        else:
            logger.info("RealDash Disconnected.")
    
    def _on_h264_subscription(self, connection, notify_enabled, indicate_enabled):
        """Handle subscription changes for H.264 stream."""
        self.h264_stream_subscribed = notify_enabled
        if notify_enabled:
            logger.info("H.264 Client Connected! Ready to stream.")
            logger.info("Send 'START' to begin streaming")
            self.h264_streaming_active = True
            self.h264_source.reload_frames()
        else:
            logger.info("H.264 Client Disconnected.")
            self.h264_streaming_active = False
    
    async def send_realdash_data(self):
        """Send RealDash CAN frames."""
        if not self.nus_tx_subscribed:
            return
        
        # Read sensor data
        temp = self.sensor_reader.read_temp()
        if temp == -999.0:
            temp = 0.0
        
        # Build frames
        frames = []
        frames.append(build_realdash_frame(3200, temp))
        
        for pos, can_id in [('FL', 3201), ('FR', 3202), ('RL', 3203), ('RR', 3204)]:
            tpms = self.tpms_data.get(pos)
            frames.append(build_tpms_frame(can_id, tpms['pressure'], tpms['temp'], 
                                          tpms['leaking'], tpms['battery']))
        
        # Send all frames
        for frame in frames:
            await self.device.notify_subscribers(self.nus_tx_char, frame)
        
        fl = self.tpms_data.get('FL')
        fr = self.tpms_data.get('FR')
        rl = self.tpms_data.get('RL')
        rr = self.tpms_data.get('RR')
        print(f"\rFrame {self.counter}: Diff={temp:.1f}C FL={fl['pressure']:.2f}bar "
              f"FR={fr['pressure']:.2f}bar RL={rl['pressure']:.2f}bar RR={rr['pressure']:.2f}bar  ", end="")
        self.counter += 1
    
    async def send_h264_frame(self):
        """Send H.264 video frame."""
        if not self.h264_stream_subscribed or not self.h264_streaming_active:
            return
        
        if not self.h264_source.has_frames():
            return
        
        try:
            # Get packetized frame from source
            packets = self.h264_source.get_next_frame_packets()
            
            if not packets:
                return
            
            # Send each packet as notification
            for packet in packets:
                await self.device.notify_subscribers(self.h264_stream_char, packet)
            
            # Get stats for display
            stats = self.h264_source.get_stats()
            print(f"\rH.264: Frame {stats['current_frame']}/{stats['total_frames']} "
                  f"({len(packets)} packets) Total={stats['total_packets_sent']} ", end="")
            
        except Exception as e:
            logger.error(f"H.264 Error: {e}")

TRANSPORT="hci-socket:0"
DEVICE_NAME="22B_Sensors"

async def main():
    # Initialize state objects
    sensor_reader = SensorReader()
    tpms_data = TPMSData()
    
    # Choose H.264 source: file-based or camera
    # h264_source = H264FileStreamSource(frames_dir="data/samples/h264")
    h264_source = H264CameraStreamSource(device='/dev/video0', width=640, height=360, framerate=30, bitrate=300000)
    
    # Open transport
    async with await open_transport(TRANSPORT) as hci_transport:
        logger.info("Transport opened.")
        
        # Create device with HCI transport
        device = Device.with_hci(
            DEVICE_NAME,
            'F0:F1:F2:F3:F4:F5',
            hci_transport.source,
            hci_transport.sink
        )
        
        # Create GATT server with dependencies
        gatt_server = BumbleGATTServer(device, sensor_reader, tpms_data, h264_source)
        
        # Debug print GATT attributes
        logger.info("GATT Server Attributes:")
        for attribute in device.gatt_server.attributes:
            logger.info(f"  {attribute}")
        
        # Prepare legacy advertising payloads per Bumble examples
        device.advertising_data = bytes(
            AdvertisingData([
                data_types.Flags(0x02),  # General Discoverable Mode
                data_types.CompleteListOf128BitServiceUUIDs([UUID(NUS_SERVICE_UUID)]),
                data_types.Appearance(
                        data_types.Appearance.Category.WEARABLE_AUDIO_DEVICE,
                        data_types.Appearance.WearableAudioDeviceSubcategory.EARBUD,
                    )
            ])
        )
        device.scan_response_data = bytes(
            AdvertisingData([
                data_types.CompleteLocalName(DEVICE_NAME),
                data_types.Appearance(
                        data_types.Appearance.Category.WEARABLE_AUDIO_DEVICE,
                        data_types.Appearance.WearableAudioDeviceSubcategory.EARBUD,
                    )
            ])
        )

        # Power on the device
        logger.info("Powering on device...")
        await device.power_on()
        logger.info("Device powered on.")
        
        # Start legacy connectable + scannable advertising (shows Flags & services)
        logger.info("Starting legacy advertising (connectable + scannable)...")
        await device.start_advertising(
            advertising_type=AdvertisingType.UNDIRECTED_CONNECTABLE_SCANNABLE,
            auto_restart=True
        )
        logger.info(f"Advertising '{DEVICE_NAME}' with Flags + NUS UUID; name/appearance in scan response.")
        
        # Start TPMS scanner
        asyncio.create_task(tpms_scanner_loop(tpms_data))
        
        # Main loop
        try:
            while True:
                # Send RealDash data every 1 second
                await gatt_server.send_realdash_data()
                # await asyncio.sleep(1)
                
                # Send H.264 frames at ~30 FPS (33ms per frame)
                await gatt_server.send_h264_frame()
                await asyncio.sleep(0.001)
                
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            # Cleanup H.264 source
            h264_source.cleanup()
        
        # Wait for termination
        await hci_transport.source.wait_for_termination()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())
