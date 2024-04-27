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
    crc = crc_x25(message[1:])
    message += [crc & 0xFF, crc >> 8]
    message_bytes = bytes(message)
    wpi.serialFlush(serial)
    # print(f"Sending: {message_bytes.hex()}")  # Display hex representation of what is being sent
    for byte in message_bytes:
        wpi.serialPutchar(serial, byte)
    time.sleep(0.2)


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
    process_gimbal_response(serial)


def read_response(serial, timeout=0.5):
    end_time = time.time() + timeout
    response = []
    while time.time() < end_time:
        while wpi.serialDataAvail(serial):
            response.append(wpi.serialGetchar(serial))
        if len(response) >= 6:  # Minimum length of a complete message
            break
        time.sleep(0.05)  # Give a short delay to allow data to arrive
    return response


def interpret_response(response):
    if len(response) < 6:
        return f"Incomplete response: {response}"
    
    # Try to parse the response
    try:
        start_byte, data_length, cmd_ack, data_byte, crc_low, crc_high = response[:6]
    # Continue parsing if more bytes are needed based on the response
    except ValueError:
        return f"Error parsing response: {response}"
    
    # Unpack the response bytes
    start_byte, data_length, cmd_ack, data_byte, crc_low, crc_high = response
    
    # Verify start byte and CMD_ACK
    if start_byte != 0xFB or cmd_ack != 0x96:
        return "Invalid response or command"

    # Calculate CRC
    expected_crc = crc_x25(response[1:-2])  # Calculate expected CRC
    received_crc = crc_low + (crc_high << 8)
    
    if expected_crc != received_crc:
        return "CRC mismatch"

    # Map data byte to message
    messages = {
        0: "ACK_OK",
        1: "ACK_ERR_FAIL",
        2: "ACK_ERR_ACCESS_DENIED",
        3: "ACK_ERR_NOT_SUPPORTED",
        150: "ACK_ERR_TIMEOUT",
        151: "ACK_ERR_CRC",
        152: "ACK_ERR_PAYLOADLEN"
    }
    return messages.get(data_byte, "Unknown error code")

def process_gimbal_response(serial):
    response_bytes = read_response(serial)
    message = interpret_response(response_bytes)
    print(f"Response: {message}")
    # Debugging: Print raw response to see what's being received
    print(f"Raw Response Bytes: {response_bytes}")


# Command-line interface to accept user input
while True:
    try:
        input_str = input("Enter pitch and yaw angles separated by a space, or 'exit' to quit: ")
        if input_str.lower() == 'exit':
            break
        pitch, yaw = map(float, input_str.split())
        set_angles(pitch, yaw)
    except ValueError:
        print("Please enter valid floating-point numbers for pitch and yaw.")
    except KeyboardInterrupt:
        print("\nExiting...")
        break

# Clean up
wpi.serialClose(serial)

def normalize_adc_value(value, deadband=100):
    # Shift the range so that the midpoint is at zero
    normalized_value = value - ADC_MID
    # Apply deadband
    if -deadband <= normalized_value <= deadband:
        return 0
    return normalized_value

# The pin number must be set according to the WiringPi setup
adc_pin1 = 29 
adc_pin2 = 25

# Constants for ADC range
ADC_MIN = 0
ADC_MAX = 4095
ADC_MID = (ADC_MAX - ADC_MIN + 1) // 2

# Global variables to store current gimbal angles
current_pitch = 0.0
current_yaw = 0.0



# # Example usage within an interactive loop
# try:
#     while True:
#         # Simulate joystick input or replace with actual ADC read values
#         input_str = input("Enter pitch and yaw increments, or 'exit' to quit: ")
#         if input_str.lower() == 'exit':
#             break
#         pitch_inc, yaw_inc = map(float, input_str.split())
#         set_angles(pitch_inc, yaw_inc)
# except KeyboardInterrupt:
#     print("\nExiting...")


# try:
#     while True:
#         analog_value1 = wpi.analogRead(adc_pin1)
#         analog_value2 = wpi.analogRead(adc_pin2)
#         print(f"Current value on Y: {normalize_adc_value(analog_value1)} X: {normalize_adc_value(analog_value2)}")

# except KeyboardInterrupt:
#     print("\nExiting...")

