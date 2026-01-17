import smbus2
import time
import os

# --- SETUP ---
# Right Side = Bus 1 (Hardware I2C)
# Left Side  = Bus 3 (Software I2C on Pins 29/31)
bus_R = smbus2.SMBus(1)
bus_L = smbus2.SMBus(3)

AS5600_ADDR = 0x36

def read_angle(bus, name):
    try:
        high = bus.read_byte_data(AS5600_ADDR, 0x0C)
        low = bus.read_byte_data(AS5600_ADDR, 0x0D)
        angle = (high << 8) | low
        return angle
    except Exception as e:
        return "ERR"

print("--- DUAL ENCODER TEST ---")
print("Spin LEFT wheel then RIGHT wheel.")

while True:
    val_L = read_angle(bus_L, "Left")
    val_R = read_angle(bus_R, "Right")
    
    # Simple visualizer
    print(f"LEFT (Bus 3): {val_L:<10} | RIGHT (Bus 1): {val_R:<10}")
    
    time.sleep(0.1)
