import smbus2
import time
import os

# RPi 5 I2C Bus 1
bus = smbus2.SMBus(1)
AS5600_ADDR = 0x36

print("--- MAGNET TEST START ---")
print("Spin the wheel manually. Look for changing numbers.")

while True:
    try:
        # Read Raw Angle (0 - 4095)
        # Register 0x0C (High Byte) and 0x0D (Low Byte)
        high = bus.read_byte_data(AS5600_ADDR, 0x0C)
        low = bus.read_byte_data(AS5600_ADDR, 0x0D)

        # Combine them
        raw_angle = (high << 8) | low

        # Read Status Register 0x0B
        # Bit 5 (MH) = Magnet too Strong (Too close)
        # Bit 4 (ML) = Magnet too Weak (Too far)
        # Bit 3 (MD) = Magnet Detected
        status = bus.read_byte_data(AS5600_ADDR, 0x0B)

        magnet_detected = (status >> 5) & 1
        magnet_too_weak = (status >> 4) & 1
        magnet_too_strong = (status >> 3) & 1

        os.system('clear')
        print(f"Raw Angle: {raw_angle} (Range: 0-4095)")
        print(f"Status: Detected={magnet_detected} | Weak={magnet_too_weak} | Strong={magnet_too_strong}")
        print("-" * 30)

        time.sleep(0.1)

    except Exception as e:
        print(f"Read Error: {e}")
        time.sleep(1)
