from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# --- PINS FROM YOUR CODE ---
# Left
LEFT_ENA = 13  # Pin 33
LEFT_IN1 = 17  # Pin 11
LEFT_IN2 = 27  # Pin 13
# Right
RIGHT_IN3 = 22 # Pin 15
RIGHT_IN4 = 10 # Pin 19
RIGHT_ENB = 19 # Pin 35

print("Initializing GPIO...")
try:
    # Devices
    pwm_left = PWMOutputDevice(LEFT_ENA)
    in1_left = DigitalOutputDevice(LEFT_IN1)
    in2_left = DigitalOutputDevice(LEFT_IN2)

    pwm_right = PWMOutputDevice(RIGHT_ENB)
    in3_right = DigitalOutputDevice(RIGHT_IN3)
    in4_right = DigitalOutputDevice(RIGHT_IN4)

    # TEST LEFT
    print("Testing LEFT Motor...")
    in1_left.on()
    in2_left.off()
    pwm_left.value = 1.0  # Full Speed
    sleep(2)
    pwm_left.value = 0

    # TEST RIGHT
    print("Testing RIGHT Motor...")
    in3_right.on()
    in4_right.off()
    pwm_right.value = 1.0  # Full Speed
    sleep(2)
    pwm_right.value = 0

    print("Done.")

except Exception as e:
    print(f"ERROR: {e}")
