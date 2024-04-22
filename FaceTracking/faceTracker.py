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

# Initialize current angles; you might want to align these with the actual initial position of your gimbal
current_pitch = 0.0
current_yaw = 0.0

def gimbal_control(stream, center_x, center_y, distance):
    global current_pitch, current_yaw

    frame_center_x = stream.width // 2
    frame_center_y = stream.height // 2

    # Calculate error as a difference between the center of the detected face and the center of the frame
    error_x = center_x - frame_center_x
    error_y = center_y - frame_center_y


    # Convert error to angle adjustments; using a factor to scale the error to a suitable angle change
    # You may need to adjust these factors based on the responsiveness and range of your gimbal
    pitch_change = error_y / 150.0
    
    yaw_change = error_x / 50.0

    # Update current angles by adding the calculated changes
    current_pitch += pitch_change
    current_yaw += round(-yaw_change, 3 )

    # Damping factor
    damping = 0.9

    # Keep angles within a reasonable range if necessary, especially if your gimbal has limited movement range
    current_pitch = round(max(min(current_pitch, 45), -45) * damping, 3)  # Assuming the gimbal can tilt between -90 and 90 degrees
    # current_yaw = round(max(min(current_yaw, 180), -180) * damping, 3) # Assuming the gimbal can pan between -90 and 90 degrees

    # print(f"Adjusting to pitch: {current_pitch}, yaw: {current_yaw}")
    # print(f"Face center at X: {center_x}, Y: {center_y}, Distance: {distance:.2f} cm")

    print(f"Box Difference X: {error_x}, Y: {error_y}  Gimbal change X:{yaw_change}, current yaw postion {current_yaw}")

    # Set the new angles
    set_angles(-5, current_yaw)


class WebcamVideoStream:
    def __init__(self, CAM_SRC=0, CAM_WIDTH=640, CAM_HEIGHT=480):
        self.stream = cv2.VideoCapture(CAM_SRC)
        self.stream.set(3, CAM_WIDTH)
        self.stream.set(4, CAM_HEIGHT)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.width = CAM_WIDTH
        self.height = CAM_HEIGHT

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
    def __init__(self, frame_queue, control_queue, cascade_path, focal_length, real_face_width=17):
        self.frame_queue = frame_queue
        self.control_queue = control_queue  # Added control queue here
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
            # print("show_FPS - Processing at %.2f fps last %i frames" % (FPS, self.fps_count))
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
                    distance = round(self.calculate_distance(w),2)
                    # print(f"Face center at X: {center_x}, Y: {center_y}, Distance: {distance:.2f} cm")
                    # Put the center coordinates in the control queue for the gimbal control to use
                    self.control_queue.put((center_x, center_y, distance))

                    # Display the distance on the frame
                    cv2.putText(frame, f"Distance: {distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

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
    control_queue = queue.Queue()
    focal_length = 1000

    video_stream = WebcamVideoStream().start()
    cascade_path = "/home/odroid/Desktop/Object_Detection_Files/FaceTracking/cascades/haarcascade_frontalface_default.xml"
    face_detection = FaceDetection(frame_queue, control_queue, cascade_path, focal_length).start()

    while True:
        frame = video_stream.read()
        if frame is None or face_detection.stopped:
            break

        if not frame_queue.full():
            frame_queue.put(frame)

        if not control_queue.empty():
            center_x, center_y, distance = control_queue.get()
            gimbal_control(video_stream, center_x, center_y, distance)  # Pass center to function


    video_stream.stop()
    face_detection.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
