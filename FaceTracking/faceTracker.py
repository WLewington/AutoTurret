import cv2
import time
import odroid_wiringpi as wpi
import threading
import struct
import queue

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
    def __init__(self, CAM_SRC=0, CAM_WIDTH=640, CAM_HEIGHT=480):
        self.stream = cv2.VideoCapture(CAM_SRC)
        self.stream.set(3, CAM_WIDTH)
        self.stream.set(4, CAM_HEIGHT)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        t = threading.Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return
            self.grabbed, self.frame = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


class FaceDetection:
    def __init__(self, frame_queue, cascade_path, focal_length, real_face_width=15):
        self.frame_queue = frame_queue
        self.cascade = cv2.CascadeClassifier(cascade_path)
        self.stopped = False
        self.fps_start_time = time.time()
        self.fps_count = 0
        self.debug = True
        self.focal_length = focal_length
        self.real_face_width = real_face_width

    def start(self):
        t = threading.Thread(target=self.detect_faces)
        t.daemon = True
        t.start()
        return self

    def show_FPS(self):
        FRAME_COUNTER = 10
        self.fps_count += 1
        if self.fps_count >= FRAME_COUNTER:
            duration = float(time.time() - self.fps_start_time)
            FPS = float(FRAME_COUNTER / duration)
            print("show_FPS - Processing at %.2f fps last %i frames" % (FPS, self.fps_count))
            self.fps_count = 0
            self.fps_start_time = time.time()

    def detect_faces(self):
        while not self.stopped:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    distance = self.calculate_distance(w)
                    print(f"Face center at X: {center_x}, Y: {center_y}, Distance: {distance:.2f} cm")
                cv2.imshow('Face Detection', frame)
                self.show_FPS()
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stopped = True

    def calculate_distance(self, face_width_pixels):
        return (self.focal_length * self.real_face_width) / face_width_pixels

    def stop(self):
        self.stopped = True


def main():

    frame_queue = queue.Queue(maxsize=10)
    focal_length = 500  # This is an example value; you'll need to calibrate this for your camera
   
    video_stream = WebcamVideoStream().start()
    # Load the pre-trained Haar Cascade for face detection
    cascade_path = "/home/odroid/Desktop/Object_Detection_Files/FaceTracking/cascades/haarcascade_frontalface_default.xml"
    face_detection = FaceDetection(frame_queue, cascade_path, focal_length).start()


    while True:
        frame = video_stream.read()
        if frame is None or face_detection.stopped:
            break

        if not frame_queue.full():
            frame_queue.put(frame)

    video_stream.stop()
    face_detection.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()