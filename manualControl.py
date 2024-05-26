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
    print(f"Sending: {message_bytes.hex()}")  # Display hex representation of what is being sent
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

def update_angles(pitch_delta, yaw_delta):
    global current_pitch, current_yaw
    # Update current angles by adding deltas
    current_pitch += pitch_delta
    current_yaw += yaw_delta

    # Ensure the angles stay within a sensible range, e.g., -120 to 120 degrees
    current_pitch = round(max(min(current_pitch, 120), -120), 2)
    current_yaw = round(max(min(current_yaw, 120), -120), 2)

    # Convert float angles to bytes
    pitch_bytes = struct.pack("<f", current_pitch)
    roll_bytes = struct.pack("<f", 0)
    yaw_bytes = struct.pack("<f", current_yaw)
    flags_byte = b'\x00'
    type_byte = b'\x00'

    # Construct the payload
    payload = list(pitch_bytes + roll_bytes + yaw_bytes + flags_byte + type_byte)
    
    # Send command
    send_rc_command(0x11, payload)
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
    print(f"Response: {message} - Payload: {response_bytes}")

def normalize_adc_value(value, deadband=100):
    # Shift the range so that the midpoint is at zero
    normalized_value = value - ADC_MID
    # Apply deadband
    if -deadband <= normalized_value <= deadband:
        return 0
    return normalized_value

def convert_int_to_float(value, limit):
    # Check if the input value is within the expected range
    if not -2048 <= value <= 2048:
        raise ValueError("Input value must be between -2047 and 2047.")

    # Define the conversion factors based on the ranges
    min_int, max_int = -2048, 2048
    min_float, max_float = -limit, limit

    # Perform the conversion
    # Formula: new_value = (old_value - old_min) / (old_max - old_min) * (new_max - new_min) + new_min
    converted_float = (value - min_int) / (max_int - min_int) * (max_float - min_float) + min_float

    # Return the value rounded to two decimal places
    return round(converted_float, 2)

# Function to read the pin value
def read_switch(pin):
    input_state = wpi.digitalRead(pin)
    if input_state == wpi.LOW:
        print("Switch Pressed")
    else:
        print("Switch Released")

# The pin number must be set according to the WiringPi setup
adc_pin1 = 29 
adc_pin2 = 25
sw_pin   = 26

# Constants for ADC range
ADC_MIN = 0
ADC_MAX = 4095
ADC_MID = (ADC_MAX - ADC_MIN + 1) // 2

# Global variables to store current gimbal angles
current_pitch = 0.0
current_yaw = 0.0

# Setting up joystick switch.
wpi.pinMode(sw_pin, wpi.INPUT_PULLUP)


try:
    while True:
        analog_value1 = wpi.analogRead(adc_pin1)
        analog_value2 = wpi.analogRead(adc_pin2)

        # input_state = wpi.digitalRead(sw_pin)
        # print(input_state)
        read_switch(sw_pin)
        
        # Normalize values and convert to float
        yaw_change = convert_int_to_float(normalize_adc_value(analog_value2), 120)  # Assuming deadband of 50
        pitch_change = convert_int_to_float(normalize_adc_value(analog_value1), 30)  # Assuming deadband of 50

        # Check if there is significant movement
        if abs(yaw_change) > 0.1 or abs(pitch_change) > 0.1:
            print(f"Applying Changes: Pitch {pitch_change}, Yaw {yaw_change}")
            update_angles(pitch_change, yaw_change)
        else:
            print(f"No significant movement detected.")

        print(f"Current Angles: Pitch {current_pitch}, Yaw {current_yaw}")
        time.sleep(0.1)  # Short delay to reduce processing load

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    wpi.serialClose(serial)