import serial
import time

# CONFIG
PORT = '/dev/ttyACM0'  # Your Tactrix Port
BAUD = 4800            # SSM2 WAKE UP SPEED (CRITICAL)

try:
    # Open Serial Port
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} at {BAUD} baud")

    # Clear any old junk in the buffer
    ser.reset_input_buffer()

    # SSM2 INIT PACKET (Wake Up ECU)
    # [Header 80] [Dest 10] [Src F0] [Size 01] [Cmd BF] [Checksum 40]
    init_packet = b'\x80\x10\xF0\x01\xBF\x40'
    
    print(f"Sending: {init_packet.hex()}")
    ser.write(init_packet)

    # Wait for ECU to process (ECUs are slow)
    time.sleep(0.5)

    # Read EVERYTHING in the buffer
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Raw RX ({len(response)} bytes): {response.hex()}")
        
        # Analyze
        if response == init_packet:
            print("RESULT: Loopback Echo ONLY. ECU did not reply.")
        elif response.startswith(init_packet):
            real_reply = response[len(init_packet):]
            print(f"RESULT: SUCCESS! ECU Replied: {real_reply.hex()}")
        else:
            print("RESULT: Received data, but it looks messy.")
    else:
        print("RESULT: Silence. No Echo, No Reply.")

    ser.close()

except Exception as e:
    print(f"Error: {e}")