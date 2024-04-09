import cv2
import time
import odroid_wiringpi as wpi
from threading import Thread
import struct

serial = wpi.serialOpen('/dev/ttyS1', 115200)

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

def gimbal_control_loop(detector, webcam_stream):
    cam_cx = 640 // 2
    cam_cy = 480 // 2
    # 640x480
    
    while True:
        frame = webcam_stream.read()
        contours = detector.process_frame(frame)
        if contours:
            # Assuming the largest contour is our point of interest
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate gimbal angles to center detected motion
                pitch, yaw = calculate_gimbal_angles(cx, cy, cam_cx, cam_cy)
                
                # Update gimbal position - Assuming set_angles takes pitch and yaw, and roll is not used
                set_angles(pitch, yaw)
            else:
                print("No valid contour detected.")
        else:
            print("No motion detected.")
        
        time.sleep(0.1)  # Adjust based on your needs



def calculate_gimbal_angles(cx, cy, cam_cx, cam_cy):
    """
    Calculate the adjustments needed for the gimbal to center the detected motion.

    Parameters:
    - cx, cy: The center coordinates of the detected motion.
    - cam_cx, cam_cy: The center coordinates of the camera's field of view.

    Returns:
    - (pitch, yaw): The pitch and yaw adjustments needed to center the detected motion.
    """
    # Calculate the difference between the center of the detected motion and the camera's center
    dx = cam_cx - cx
    dy = cam_cy - cy

    # Convert the differences into pitch and yaw adjustments
    # The sensitivity factors (7 and 6) were used in the original script to convert coordinate differences into servo angles.
    # These factors might need adjustment based on your gimbal's specific behavior and range of motion.
    yaw = int(dx / 7)
    pitch = int(dy / 6)
    print("Yaw: " + str(yaw) + " , Pitch: " + str(pitch))

    # Ensure the pitch and yaw adjustments are within the gimbal's allowed range
    # This might require specific limits based on your gimbal's capabilities
    # yaw = max(min(yaw, max_yaw), min_yaw)
    # pitch = max(min(pitch, max_pitch), min_pitch)

    return pitch, yaw


class WebcamVideoStream:
    def __init__(self, src=0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


class MotionDetector:
    def __init__(self):
        self.prev_frame = None

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        if self.prev_frame is None:
            self.prev_frame = gray
            return None
        
        frame_diff = cv2.absdiff(self.prev_frame, gray)
        thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.prev_frame = gray
        
        return contours


class FaceDetector:
    def __init__(self):
        # Specify the full path to the Haar Cascade XML file
        cascade_path = "/home/odroid/Desktop/Object_Detection_Files/cascades/haarcascade_frontalface_default.xml"
        
        # Load the pre-trained Haar Cascade model for face detection using the specified path
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        
    def detect_faces(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)  # Draw a blue rectangle around each face
        
        return frame, faces

# Initialize WebcamVideoStream and start the capture thread
webcam_stream = WebcamVideoStream(src=0, width=640, height=480).start()

# # Initialize the MotionDetector
# motion_detector = MotionDetector()

# Initialize the FaceDetector
face_detector = FaceDetector()

# Start the gimbal control loop in a separate thread
# Thread(target=gimbal_control_loop, args=(motion_detector, webcam_stream), daemon=True).start()

try:
    while True:
        # Main loop can be used for additional tasks or to display the frames
        frame = webcam_stream.read()

        # Detect faces and draw rectangles around them
        frame_with_faces, faces = face_detector.detect_faces(frame)

        cv2.imshow("Frame", frame_with_faces)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    webcam_stream.stop()
    cv2.destroyAllWindows()