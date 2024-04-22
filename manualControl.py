import odroid_wiringpi as wpi
import struct
import time


serial = wpi.serialOpen('/dev/ttyS1', 115200)

# Initialize WiringPi in GPIO mode
wpi.wiringPiSetup()


# Function to calculate x25 CRC
def crc_x25(buffer):
    crc = 0xffff
    for b in buffer:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xffff

def send_rc_command(command, payload=[]):
    message = [0xFA, len(payload), command] + payload
    crc = crc_x25(message[1:])  # CRC excludes start sign
    message += [crc & 0xFF, crc >> 8]  # Append CRC as low and high bytes
    message_bytes = bytes(message)
    wpi.serialFlush(serial)
    for byte in message_bytes:
        wpi.serialPutchar(serial, byte)  # Send each byte individually
    time.sleep(0.1)  # Give time for response
    
    output_str = ''
    while wpi.serialDataAvail(serial):
        output_str += chr(wpi.serialGetchar(serial))
    # print(f"Response: {output_str}")

def set_angles(pitch, yaw):
    # Convert float angles to bytes
    pitch_bytes = struct.pack("<f", pitch)
    roll_bytes = struct.pack("<f", 0)
    yaw_bytes = struct.pack("<f", yaw)
    flags_byte = b'\x00'
    type_byte = b'\x00'

    # Construct the payload
    payload = pitch_bytes + roll_bytes + yaw_bytes + flags_byte + type_byte
    
    # Send command
    send_rc_command(0x11, list(payload))



# # Command-line interface to accept user input
# while True:
#     try:
#         input_str = input("Enter pitch and yaw angles separated by a space, or 'exit' to quit: ")
#         if input_str.lower() == 'exit':
#             break
#         pitch, yaw = map(float, input_str.split())
#         set_angles(pitch, yaw)
#     except ValueError:
#         print("Please enter valid floating-point numbers for pitch and yaw.")
#     except KeyboardInterrupt:
#         print("\nExiting...")
#         break

# # Clean up
# wpi.serialClose(serial)


# The pin number must be set according to the WiringPi setup
adc_pin1 = 29  # This is a placeholder; the pin number may be different
adc_pin2 = 25  # This is a placeholder; the pin number may be different

# Constants for ADC range
ADC_MIN = 0
ADC_MAX = 4095
ADC_MID = (ADC_MAX - ADC_MIN) // 2


def read_adc(pin):
    # Read the analog value from the ADC pin
    analog_value = wpi.analogRead(pin)
    # Map the value to the new range with midpoint at zero
    mapped_value = (analog_value - ADC_MID)
    return mapped_value

while True:
    try:
        # Read the mapped analog values from both ADC pins
        mapped_value1 = read_adc(adc_pin1)
        mapped_value2 = read_adc(adc_pin2)
        
        # print(f"Mapped analog value on pin {adc_pin1}: {mapped_value1}")
        # print(f"Mapped analog value on pin {adc_pin2}: {mapped_value2}")
        print(f"Analog value on Y: {adc_pin1}: {mapped_value1}, X: {mapped_value2}")
        
    except KeyboardInterrupt:
        print("\nExiting...")
        break


