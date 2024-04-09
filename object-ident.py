import cv2
import odroid_wiringpi as wpi
import time
import struct
import threading


serial = wpi.serialOpen('/dev/ttyS1', 115200)
frame_center = (320, 240)  # Assuming a 640x480 frame, this is the center
current_pitch = 0  # Initial pitch
current_yaw = 0  # Initial yaw

# Initialize variables for calculating FPS
frame_count = 0
start_time = time.time()

def print_fps():
    global frame_count, start_time
    frame_count += 1
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time >= 1:  # Update every second
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        frame_count = 0
        start_time = time.time()

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
    print(f"Response: {output_str}")

def set_angles(pitch, roll, yaw):
    # Convert float angles to bytes
    pitch_bytes = struct.pack("<f", pitch)
    roll_bytes = struct.pack("<f", roll)
    yaw_bytes = struct.pack("<f", yaw)
    flags_byte = b'\x00'
    type_byte = b'\x00'

    # Construct the payload
    payload = pitch_bytes + roll_bytes + yaw_bytes + flags_byte + type_byte
    
    # Send command
    send_rc_command(0x11, list(payload))


def handle_angles_input():
    while True:
        user_input = input("Enter angles command (e.g., 'angles 10.0 -5.0 0.0'): ")
        if user_input == "quit":
            # Close the serial connection
            wpi.serialClose(serial)
            break
        try:
            command, pitch_str, roll_str, yaw_str = user_input.split()
            if command == "angles":
                pitch = float(pitch_str)
                roll = float(roll_str)
                yaw = float(yaw_str)
                set_angles(pitch, roll, yaw)
            else:
                print("Invalid command. Please use the 'angles' command.")
        except ValueError:
            print("Invalid input. Please enter commands like 'angles 10.0 -5.0 0.0'.")


def center_object(frame_center, object_center, current_pitch, current_yaw):
    PIXELS_PER_DEGREE_YAW = 640 / 70  # Assuming a 70 degree horizontal FOV
    PIXELS_PER_DEGREE_PITCH = 480 / 70  # Assuming a 70 degree vertical FOV, adjust as necessary
    YAW_THRESHOLD = 1  # Minimum yaw change in degrees to send a new command
    PITCH_THRESHOLD = 1  # Minimum pitch change in degrees to send a new command

    # Get the x and y-coordinate differences between object center and frame center
    dx_pixels = object_center[0] - frame_center[0]
    dy_pixels = object_center[1] - frame_center[1]

    # Calculate the yaw and pitch adjustments in degrees
    yaw_adjustment = dx_pixels / PIXELS_PER_DEGREE_YAW
    pitch_adjustment = dy_pixels / PIXELS_PER_DEGREE_PITCH

    DAMPING_FACTOR = 0.2  # Adjust this value based on testing
    # Calculate the new yaw and pitch with damping
    new_yaw = current_yaw - (yaw_adjustment * DAMPING_FACTOR)
    new_pitch = current_pitch + (pitch_adjustment * DAMPING_FACTOR)

    print(f"Current yaw: {current_yaw:.2f}, Yaw adjustment: {yaw_adjustment:.2f}, New yaw: {new_yaw:.2f}")
    print(f"Current pitch: {current_pitch:.2f}, Pitch adjustment: {pitch_adjustment:.2f}, New pitch: {new_pitch:.2f}")

    # Check if the yaw change is significant enough to warrant a command
    if abs(yaw_adjustment) > YAW_THRESHOLD or abs(pitch_adjustment) > PITCH_THRESHOLD:
        print("Significant yaw or pitch change detected. Sending new command.")
        set_angles(new_pitch, 0, new_yaw)  # Assuming roll is not used and thus set to 0
        return new_pitch, new_yaw
    else:
        # If the changes are too small, don't send a new command
        print("Minor yaw or pitch change detected. No command sent.")
        return current_pitch, current_yaw






classNames = []
classFile = "/home/odroid/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/odroid/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/odroid/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def getObjects(img, thres, nms, draw=True, objects=[]):
    global current_pitch, current_yaw  # Add this line to use the global variables

    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    # Print position and class name to the console

                # print(f"Detected {className} at x: {box[0]}, y: {box[1]} Confidence: {confidence*100:.2f}%")
                # Correctly calculate the center of the detected object using 'box'
                object_center = (box[0] + box[2] // 2, box[1] + box[3] // 2)
                # Call center_object to adjust gimbal
                current_pitch, current_yaw = center_object(frame_center, object_center, current_pitch, current_yaw)

    return img,objectInfo


if __name__ == "__main__":

    # input_thread = threading.Thread(target=handle_angles_input)
    # input_thread.daemon = True  # This ensures the thread will exit when the main program does
    # input_thread.start()

    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    #cap.set(10,70)



    while True:

        success, img = cap.read()
        # Settings for tweaking what objects to look for and what certenty. 
        result, objectInfo = getObjects(img, 0.55, 0.2, objects=['person'])

        # Call the function to print FPS
        print_fps()

        #print(objectInfo)
        cv2.imshow("Output",img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
