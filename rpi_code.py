#!/usr/bin/env python3
"""
Hey! This is the Raspberry Pi vision code for our waste sorting robot.
It handles all the computer vision stuff - detecting objects, finding bins,
figuring out where to go, and sending commands to our Arduino buddy.

We've kept things simple by using a unified approach for handling
both objects and recycling bins.
"""

import cv2                # For all the computer vision magic
import time               # For timing and delays
import math               # For angle calculations
import numpy as np        # Math on steroids
import serial             # For talking to Arduino
from threading import Thread  # For doing multiple things at once
import imutils            # Some handy image processing utilities
from picamera2 import Picamera2  # For the Pi camera

# YOLO model for detecting objects
from ultralytics import YOLO

# Some important constants we tuned through testing
FRAME_WIDTH = 500
FRAME_HEIGHT = 400
CAMERA_CENTER_X = 275
CAMERA_CENTER_Y = 370
CONFIDENCE_THRESHOLD = 0.5
OBJECT_FRAME_COUNT = 1  # How many frames in a row to confirm detection
ARUCO_DICT = cv2.aruco.DICT_6X6_250  # Type of ArUco markers we're using

# Start in manual mode by default
AUTO_MODE = False

# Arduino communication settings
SERIAL_PORT = '/dev/ttyAMA0'  # UART port on the Pi
BAUD_RATE = 115200

# How precisely we need to align with targets
ROUGH_ALIGNMENT_THRESHOLD = 5.0  # Good enough for rough alignment
FINE_ALIGNMENT_THRESHOLD = 4.0   # Precision needed for fine alignment

# Our main targets - what the robot is looking for
class Target:
    OBJECT = 0  # Looking for recyclable items
    BIN = 1     # Looking for bins to deposit items

# Robot brain states - keeps track of what we're doing
class State:
    MANUAL = -1      # Human driving mode
    SEARCH = 0       # Looking around for targets
    ALIGN_ROUGH = 1  # Roughly turning toward target
    ALIGN_FINE = 2   # Fine-tuning our alignment
    APPROACH = 3     # Moving forward to target
    ACTION = 4       # Grabbing or dropping objects
    TRANSITION = 5   # Switching between tasks
    COMPLETE = 6     # All done!

# Global variables to keep track of everything
current_target = Target.OBJECT
current_state = State.MANUAL
waiting_for_arduino = False
consecutive_detections = 0
frame = None
detection_data = None
last_distance = 0  # Last distance reading from Arduino
detection_active = False  # Enable/disable detection processing
aruco_detection_locked = False  # Flag to completely disable ArUco detection
last_speed_update_time = 0  # For throttling speed updates
target_aruco_id = 0  # Which bin we're targeting (ArUco ID)
detected_object_class = -1  # Type of object we found (-1 = none)

# Map objects to the right bins
OBJECT_TO_BIN_MAP = {
    0: 2,  # Metal -> ArUco ID 2
    1: 0,  # Plastic -> ArUco ID 0
    2: 1,  # Paper -> ArUco ID 1
}

# Fire up the Pi camera
def initialize_camera():
    print("Waking up the camera...")
    picam2 = Picamera2()
    
    # Set up a higher resolution for better vision
    camera_config = picam2.create_preview_configuration(
        main={"size": (1280, 960), "format": "RGB888"}
    )
    picam2.configure(camera_config)
    
    # Start it up
    picam2.start()
    
    # Give it a moment to warm up
    time.sleep(2.0)
    
    return picam2

# Load up our trained YOLO model
def initialize_yolo():
    print("Loading our AI brain (YOLO model)...")
    model = YOLO('best_ncnn_model', task='detect')
    return model

# Connect to Arduino via serial
def initialize_serial():
    print(f"Opening chat line to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2.0)  # Give serial connection time to settle
        return ser
    except Exception as e:
        print(f"Uh oh! Couldn't talk to Arduino: {e}")
        exit(1)

# Send a command to Arduino
def send_command(ser, command):
    global waiting_for_arduino
    print(f"Telling Arduino: {command}")
    ser.write(f"{command}\n".encode('utf-8'))
    waiting_for_arduino = True

# Wait for Arduino to respond
def wait_for_response(ser, timeout=10.0):
    global waiting_for_arduino, last_distance
    print("Waiting for Arduino to respond...")
    start_time = time.time()
    while waiting_for_arduino and (time.time() - start_time < timeout):
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"Arduino says: {response}")
            
            # Check if Arduino is telling us a distance
            if "Distance:" in response:
                try:
                    distance_str = response.split("Distance:")[1].strip()
                    distance_value = float(distance_str.split()[0])
                    last_distance = distance_value
                    print(f"Updated distance: {last_distance:.1f} cm")
                except Exception as e:
                    print(f"Couldn't understand the distance reading: {e}")
            
            if "COMPLETE" in response or "RECEIVED" in response or "REACHED" in response or "STARTED" in response:
                waiting_for_arduino = False
                return response
        time.sleep(0.1)
    
    if waiting_for_arduino:
        print("Arduino isn't responding - did it crash?")
        waiting_for_arduino = False
    
    return None

# Figure out which way to turn to face the object
def calculate_angle(object_x, object_y):
    # Find the vector from camera center to object
    vector_x = object_x - CAMERA_CENTER_X
    vector_y = CAMERA_CENTER_Y - object_y  # Y is flipped in image coordinates
    
    # Get the angle in degrees (0 is straight ahead)
    angle = math.degrees(math.atan2(vector_x, vector_y))
    
    return angle

# Run our AI to detect recyclable objects
def run_yolo_detection(model, frame, confidence):
    try:
        # Make sure we have an actual image
        if frame is None:
            print("Hmm, got an empty frame for detection")
            return []
            
        # Let YOLO work its magic
        results = model(frame, conf=confidence)[0]
        
        detections = []
        # Process what YOLO found
        for detection in results.boxes.data.tolist():
            # Extract all the juicy details [x1, y1, x2, y2, confidence, class_id]
            x1, y1, x2, y2, conf, class_id = detection
            
            if conf >= confidence:
                # Find the center of the object
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Store everything we need to know
                detections.append({
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'confidence': conf,
                    'center': (center_x, center_y),
                    'class_id': int(class_id)  # Remember what type of object this is
                })
        
        # Nice readable names for logging
        class_names = {0: "metal", 1: "plastic", 2: "paper"}
        class_info = ", ".join([f"{class_names[d['class_id']]}" for d in detections]) if detections else "none"
        print(f"YOLO found {len(detections)} objects: {class_info}")
        return detections
    except Exception as e:
        print(f"Oops! YOLO hit a snag: {e}")
        return []

# Set up ArUco marker detection for finding bins
def initialize_aruco():
    print("Setting up ArUco marker detector for bin recognition...")
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return aruco_dict, aruco_params

# Find ArUco markers (our bin identifiers) in the camera image
def detect_aruco_markers(frame, aruco_dict, aruco_params):
    try:
        # Convert to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find those markers!
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
        
        markers = []
        if ids is not None:
            for i, corner in enumerate(corners):
                # Get the marker's ID number
                marker_id = ids[i][0]
                
                # Calculate the center point
                corners_array = corner[0]
                center_x = np.mean([c[0] for c in corners_array])
                center_y = np.mean([c[1] for c in corners_array])
                
                markers.append({
                    'id': marker_id,
                    'corners': corner,
                    'center': (center_x, center_y)
                })
            
            print(f"Found {len(markers)} ArUco markers: {[m['id'] for m in markers]}")
        else:
            print("No bin markers in sight")
        
        return markers
    except Exception as e:
        print(f"ArUco detection hit a problem: {e}")
        return []

# Function to process video and handle detections
def video_processing_thread(picam2, model, aruco_dict, aruco_params):
    global frame, detection_data, consecutive_detections
    global current_state, current_target, detection_active
    global aruco_detection_locked, target_aruco_id, detected_object_class
    global AUTO_MODE, last_distance
    
    # Set up display window
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Feed", FRAME_WIDTH, FRAME_HEIGHT)

    while True:
        try:
            # Capture frame with picamera2
            original_frame = picam2.capture_array()
            
            # Resize frame to maintain aspect ratio
            height, width = original_frame.shape[:2]
            scaling_factor = FRAME_WIDTH / width
            new_height = int(height * scaling_factor)
            
            # Resize to maintain aspect ratio first
            resized_frame = cv2.resize(original_frame, (FRAME_WIDTH, new_height))
            
            # Then crop or pad to match FRAME_HEIGHT
            if new_height > FRAME_HEIGHT:
                start_y = (new_height - FRAME_HEIGHT) // 2
                frame = resized_frame[start_y:start_y+FRAME_HEIGHT, 0:FRAME_WIDTH]
            else:
                # If the resized image is smaller than FRAME_HEIGHT, pad it
                frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
                start_y = (FRAME_HEIGHT - new_height) // 2
                frame[start_y:start_y+new_height, 0:FRAME_WIDTH] = resized_frame
            
            # Create a clean copy of the frame for drawing
            display_frame = frame.copy()
            
            # Skip processing if not in AUTO mode, but still show the camera feed
            if not AUTO_MODE:
                # Display text on frame to show manual mode
                cv2.putText(display_frame, "MANUAL MODE", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(display_frame, "Press 'a' to enter AUTO mode", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Camera Feed", display_frame)
                
                # Process keyboard commands
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # Quit
                    break
                elif key == ord('a'):  # Toggle AUTO mode
                    print("Enabling AUTO mode via keyboard")
                    AUTO_MODE = True
                    current_state = State.SEARCH
                    current_target = Target.OBJECT
                    detected_object_class = -1  # Reset the detected object class
                    detection_active = True
                elif key == ord('1'):  # Metal
                    detected_object_class = 0
                    print(f"Manually set detected object class to METAL (class_id: {detected_object_class})")
                elif key == ord('2'):  # Plastic
                    detected_object_class = 1  
                    print(f"Manually set detected object class to PLASTIC (class_id: {detected_object_class})")
                elif key == ord('3'):  # Paper
                    detected_object_class = 2
                    print(f"Manually set detected object class to PAPER (class_id: {detected_object_class})")
                
                time.sleep(0.1)
                continue
            
            # Process frame based on current state if in AUTO mode
            if detection_active and not (aruco_detection_locked and current_target == Target.BIN):
                if current_target == Target.OBJECT:
                    # Run YOLO detection for objects
                    detections = run_yolo_detection(model, frame, CONFIDENCE_THRESHOLD)
                    
                    if detections and current_state in [State.SEARCH, State.ALIGN_ROUGH, State.ALIGN_FINE]:
                        # Take only the largest detection for simplicity
                        largest_detection = max(detections, key=lambda x: (x['bbox'][2]-x['bbox'][0])*(x['bbox'][3]-x['bbox'][1]))
                        detection_data = largest_detection
                        consecutive_detections += 1
                        
                        # Get the class ID and determine target bin
                        if 'class_id' in largest_detection:
                            detected_object_class = largest_detection['class_id']
                            class_names = {0: "metal", 1: "plastic", 2: "paper"}
                            class_name = class_names.get(detected_object_class, "unknown")
                            print(f"Object detected: {class_name} (class_id: {detected_object_class}), consecutive count: {consecutive_detections}")
                        else:
                            print(f"Object detected without class_id! Consecutive count: {consecutive_detections}")
                    else:
                        if detection_data is not None:
                            print("Lost object tracking")
                        detection_data = None
                        consecutive_detections = 0
                elif current_target == Target.BIN:  # Only process when detection_active is True AND not locked
                    # Run ArUco marker detection for bins
                    markers = detect_aruco_markers(frame, aruco_dict, aruco_params)
                    
                    # Find marker with target ID (looking for ID 0)
                    target_markers = []
                    if target_aruco_id is not None:  # Make sure target_aruco_id is defined
                        target_markers = [m for m in markers if m['id'] == target_aruco_id]
                    if target_markers:
                        # Use the first matching marker
                        detection_data = target_markers[0]
                        print(f"Found ArUco marker with ID {target_aruco_id}")
                    else:
                        if detection_data is not None:
                            print(f"Lost ArUco marker with ID {target_aruco_id}")
                        detection_data = None
            else:
                # Detection is disabled, ensure detection_data is cleared
                # to prevent stale detections from being used
                if detection_data is not None:
                    locked_status = "LOCKED" if aruco_detection_locked else "disabled"
                    print(f"Detection {locked_status} - clearing detection data")
                    detection_data = None
                    consecutive_detections = 0
                
                # If we're in APPROACH state, detection should stay disabled
                if current_state == State.APPROACH:
                    detection_active = False
            
            # Draw debugging visuals on display_frame
            if detection_data:
                if current_target == Target.OBJECT:
                    # Draw bounding box for YOLO
                    x1, y1, x2, y2 = detection_data['bbox']
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw center point and line to frame center
                    center_x, center_y = detection_data['center']
                    cv2.circle(display_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    cv2.line(display_frame, (int(center_x), int(center_y)), 
                            (CAMERA_CENTER_X, CAMERA_CENTER_Y), (255, 0, 0), 2)
                    
                    # Draw confidence and class
                    class_names = {0: "metal", 1: "plastic", 2: "paper"}
                    class_id = detection_data.get('class_id', -1)
                    class_name = class_names.get(class_id, "unknown")
                    conf_text = f"{class_name}: {detection_data['confidence']:.2f}"
                    
                    cv2.putText(display_frame, conf_text, (x1, y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Draw info about which bin will be targeted
                    if class_id in OBJECT_TO_BIN_MAP:
                        target_bin = OBJECT_TO_BIN_MAP[class_id]
                        cv2.putText(display_frame, f"Target Bin: ArUco #{target_bin}", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2)
                    
                    # Draw angle information if in alignment states
                    if current_state in [State.ALIGN_ROUGH, State.ALIGN_FINE]:
                        angle = calculate_angle(center_x, center_y)
                        cv2.putText(display_frame, f"Angle: {angle:.1f}Â°", (10, 120),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                elif current_target == Target.BIN:
                    # Draw ArUco marker
                    cv2.polylines(display_frame, [np.int32(detection_data['corners'])], True, (0, 255, 0), 2)
                    
                    # Draw center point and line to frame center
                    center_x, center_y = detection_data['center']
                    cv2.circle(display_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    cv2.line(display_frame, (int(center_x), int(center_y)), 
                            (CAMERA_CENTER_X, CAMERA_CENTER_Y), (255, 0, 0), 2)
                    
                    # Draw marker ID with bin type information
                    if 'id' in detection_data:
                        aruco_id = detection_data['id']
                        bin_types = {0: "Plastic", 1: "Paper", 2: "Metal"}
                        bin_type = bin_types.get(aruco_id, "Unknown")
                        marker_text = f"ID: {aruco_id} ({bin_type})"
                        
                        cv2.putText(display_frame, marker_text, 
                                   (int(center_x), int(center_y) - 15),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Draw angle information if in alignment states
                    if current_state in [State.ALIGN_ROUGH, State.ALIGN_FINE]:
                        angle = calculate_angle(center_x, center_y)
                        cv2.putText(display_frame, f"Angle: {angle:.1f}Â°", (10, 120),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Draw frame center
            cv2.circle(display_frame, (CAMERA_CENTER_X, CAMERA_CENTER_Y), 5, (0, 255, 255), -1)
            cv2.circle(display_frame, (CAMERA_CENTER_X, CAMERA_CENTER_Y), 50, (0, 255, 255), 1)  # Target zone
            
            # Add state information to frame
            state_names = ["SEARCH", "ALIGN_ROUGH", "ALIGN_FINE", "APPROACH", 
                          "ACTION", "TRANSITION", "COMPLETE"]
            target_names = ["OBJECT", "BIN"]
            
            # Show state and target information
            cv2.putText(display_frame, f"Target: {target_names[current_target]}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if current_state >= 0:  # Not MANUAL
                cv2.putText(display_frame, f"State: {state_names[current_state]}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show detection status
            detect_status = "Active" if detection_active else "Disabled"
            detect_type = "YOLO" if current_target == Target.OBJECT else "ArUco"
            locked_status = "(LOCKED)" if aruco_detection_locked else ""
            cv2.putText(display_frame, f"Detection: {detect_type} ({detect_status}) {locked_status}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 150), 2)
            
            # Show detected object class if available
            if detected_object_class >= 0:
                class_names = {0: "METAL", 1: "PLASTIC", 2: "PAPER"}
                class_name = class_names.get(detected_object_class, "UNKNOWN")
                
                # Use color coding for different materials
                class_colors = {0: (120, 120, 255), 1: (50, 200, 50), 2: (50, 50, 200)}
                class_color = class_colors.get(detected_object_class, (200, 200, 200))
                
                cv2.putText(display_frame, f"Detected: {class_name}", (FRAME_WIDTH - 180, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, class_color, 2)
            
            # Show target bin info if searching for a bin
            if current_target == Target.BIN and current_state == State.SEARCH:
                if target_aruco_id is not None:
                    bin_types = {0: "Plastic", 1: "Paper", 2: "Metal"}
                    bin_type = bin_types.get(target_aruco_id, "Unknown")
                    cv2.putText(display_frame, f"Looking for bin: ArUco #{target_aruco_id} ({bin_type})", (10, 210),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(display_frame, "Error: No target bin set", (10, 210),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Show distance if available
            if last_distance > 0:
                cv2.putText(display_frame, f"Distance: {last_distance:.1f} cm", (10, 180),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 255, 150), 2)
            
            # Add keyboard command instructions to display
            cv2.putText(display_frame, "Keys: q=quit, o=manual, 1/2/3=class, s=skip", 
                       (10, display_frame.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show the frame
            cv2.imshow("Camera Feed", display_frame)
            
            # Process keyboard commands
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Quit
                break
            elif key == ord('o'):  # Toggle to manual mode
                print("Switching to MANUAL mode via keyboard")
                AUTO_MODE = False
                current_state = State.MANUAL
                detection_active = False
                detected_object_class = -1  # Reset detected class
            elif key == ord('1'):  # Metal
                detected_object_class = 0
                print(f"Manually set detected object class to METAL (class_id: {detected_object_class})")
            elif key == ord('2'):  # Plastic
                detected_object_class = 1  
                print(f"Manually set detected object class to PLASTIC (class_id: {detected_object_class})")
            elif key == ord('3'):  # Paper
                detected_object_class = 2
                print(f"Manually set detected object class to PAPER (class_id: {detected_object_class})")
            elif key == ord('s'):  # Skip to next state
                print("Manually advancing to next state")
                if current_state == State.SEARCH:
                    current_state = State.ALIGN_ROUGH
                elif current_state == State.ALIGN_ROUGH:
                    current_state = State.ALIGN_FINE
                elif current_state == State.ALIGN_FINE:
                    current_state = State.APPROACH
                elif current_state == State.APPROACH:
                    current_state = State.ACTION
                elif current_state == State.ACTION:
                    current_state = State.TRANSITION
                    if current_target == Target.OBJECT:
                        current_target = Target.BIN
                        current_state = State.SEARCH
                        
                        # For manual advancement, choose bin based on detected object
                        if detected_object_class in OBJECT_TO_BIN_MAP:
                            target_aruco_id = OBJECT_TO_BIN_MAP[detected_object_class]
                            print(f"Setting target bin to ArUco #{target_aruco_id} based on detected class {detected_object_class}")
                        else:
                            target_aruco_id = 0  # Default to first bin if no object detected
                            print(f"Setting default target bin to ArUco #{target_aruco_id}")
                        
                        detection_active = True
                    else:
                        current_state = State.COMPLETE
                        detected_object_class = -1  # Reset for next cycle
                print(f"Advanced to state: {state_names[current_state]}")
            
            time.sleep(0.01)  # Small delay to prevent CPU overuse
            
        except Exception as e:
            print(f"Error in video processing thread: {e}")
            time.sleep(0.1)  # Delay to prevent error loops

# The brain that controls everything
def control_thread(ser):
    global current_state, AUTO_MODE, waiting_for_arduino
    global consecutive_detections, detection_data, current_target
    global last_distance, detection_active, aruco_detection_locked
    global target_aruco_id, detected_object_class
    
    previous_state = current_state
    
    while True:
        # Let us know when we're changing states
        if current_state != previous_state:
            state_names = ["SEARCH", "ALIGN_ROUGH", "ALIGN_FINE", "APPROACH", 
                          "ACTION", "TRANSITION", "COMPLETE"]
            if current_state >= 0:  # Not MANUAL
                print(f"STATE CHANGE: {state_names[previous_state]} -> {state_names[current_state]}")
            previous_state = current_state
        
        # MANUAL mode - just wait for commands
        if not AUTO_MODE:
            # Check if Arduino wants us to go autonomous
            if ser.in_waiting > 0:
                try:
                    command = ser.readline().decode('utf-8').strip()
                    print(f"Received command: {command}")
                    
                    if command == "AUTO_START":
                        print("Arduino says go autonomous - let's roll!")
                        AUTO_MODE = True
                        current_state = State.SEARCH
                        current_target = Target.OBJECT
                        detected_object_class = -1  # Reset our object memory
                        detection_active = True
                except Exception as e:
                    print(f"Couldn't understand Arduino: {e}")
            time.sleep(0.1)
            continue
        
        # Check for messages from Arduino
        if ser.in_waiting > 0:
            try:
                command = ser.readline().decode('utf-8').strip()
                print(f"Arduino says: {command}")
                
                # Parse any distance updates
                if "Distance:" in command:
                    try:
                        distance_str = command.split("Distance:")[1].strip()
                        distance_value = float(distance_str.split()[0])
                        last_distance = distance_value
                        
                        # Log with more context for better debugging
                        target_type = "OBJECT" if current_target == Target.OBJECT else "BIN"
                        stop_distance = "OBJECT_STOP_DISTANCE" if current_target == Target.OBJECT else "BIN_STOP_DISTANCE"
                        print(f"DISTANCE UPDATE: {last_distance:.1f} cm for {target_type}, STATE: {current_state}, DETECTION: {'ACTIVE' if detection_active else 'DISABLED'}")
                        
                        # Warn when we're getting super close
                        if last_distance < 10.0:
                            print(f"WHOA! We're really close! Distance: {last_distance:.1f} cm")
                    except Exception as e:
                        print(f"Couldn't parse the distance: {e}")
                
                if command == "AUTO_STOP":
                    AUTO_MODE = False
                    current_state = State.MANUAL
                    detection_active = False
                    continue
                elif command == "READY_FOR_BIN_SEARCH":
                    # Time to find the right bin for our object
                    print("Arduino's ready to find the recycling bin")
                    current_target = Target.BIN
                    current_state = State.SEARCH
                    
                    # Figure out which bin to target based on the object type
                    if detected_object_class in OBJECT_TO_BIN_MAP:
                        target_aruco_id = OBJECT_TO_BIN_MAP[detected_object_class]
                        class_names = {0: "metal", 1: "plastic", 2: "paper"}
                        class_name = class_names.get(detected_object_class, "unknown")
                        print(f"Looking for bin #{target_aruco_id} for {class_name} object (class_id: {detected_object_class})")
                    else:
                        # If we somehow lost track of the object type, default to the first bin
                        target_aruco_id = 0
                        print(f"Huh, not sure what we're holding. Defaulting to bin #{target_aruco_id}")
                    
                    # Make sure detection is ready for the bin search
                    aruco_detection_locked = False
                    detection_active = True
                    continue
                elif command == "DETECTION_LOCK":
                    # Arduino wants us to stop looking for ArUco markers
                    print("Locking detection - we're committed to our approach path")
                    detection_active = False
                    aruco_detection_locked = True  # Don't restart detection
                    # Clear any existing detection data
                    detection_data = None
                    consecutive_detections = 0
                    continue
                elif command == "TARGET_DISTANCE_REACHED":
                    # We've reached the object - ready for pickup
                    print("At pickup distance - Arduino's handling it from here")
                    detection_active = False
                    current_state = State.ACTION
                    continue
                elif command == "BIN_DISTANCE_REACHED":
                    # We've reached the bin - ready for dropoff
                    print("At the bin! Telling Arduino to drop the object")
                    detection_active = False
                    current_state = State.ACTION
                    # Tell Arduino to do the dropoff
                    send_command(ser, "DROPOFF")
                    wait_for_response(ser)
                    
                    # Reset everything for the next cycle
                    detected_object_class = -1
                    
                    current_state = State.COMPLETE
                    continue
            except Exception as e:
                print(f"Error processing Arduino message: {e}")
        
        # State machine - the robot's brain
        if current_state == State.SEARCH:
            if current_target == Target.OBJECT:
                # Looking for recyclables
                if consecutive_detections >= OBJECT_FRAME_COUNT:
                    print("Found something! Moving to rough alignment")
                    current_state = State.ALIGN_ROUGH
            else:  # Target.BIN
                # Make sure we know which bin we're looking for
                if target_aruco_id is None:
                    target_aruco_id = 0  # Default to first bin
                    print(f"Hmm, target bin wasn't set. Looking for bin #{target_aruco_id}")
                    
                # Look for the ArUco marker with our target ID
                if detection_data and 'id' in detection_data and detection_data['id'] == target_aruco_id:
                    print(f"Found bin #{target_aruco_id}! Starting alignment")
                    current_state = State.ALIGN_ROUGH
        
        elif current_state == State.ALIGN_ROUGH:
            if detection_data:
                center_x, center_y = detection_data['center']
                angle = calculate_angle(center_x, center_y)
                print(f"Target is at angle: {angle:.1f} degrees")
                
                # Skip to fine alignment if we're already pretty close
                if abs(angle) < ROUGH_ALIGNMENT_THRESHOLD:
                    print("We're already roughly aligned - moving to fine alignment")
                    current_state = State.ALIGN_FINE
                else:
                    # Tell Arduino to turn toward target
                    print(f"Turning {angle:.1f} degrees to get roughly aligned")
                    send_command(ser, f"TURN:{angle}")
                    wait_for_response(ser)
                    
                    # Give time for the turn to settle
                    time.sleep(3.0)
                    
                    # Now fine-tune our alignment
                    current_state = State.ALIGN_FINE
            else:
                # Uh oh, lost our target
                print(f"Lost sight of {current_target} during rough alignment - back to search")
                current_state = State.SEARCH
        
        elif current_state == State.ALIGN_FINE:
            if detection_data:
                center_x, center_y = detection_data['center']
                angle = calculate_angle(center_x, center_y)
                print(f"Fine alignment check - target at angle: {angle:.1f} degrees")
                
                # Good enough to start approaching?
                if abs(angle) < FINE_ALIGNMENT_THRESHOLD:
                    print(f"Alignment looks good (angle: {angle:.1f}°) - starting approach")
                    # Stop detection during approach so it doesn't distract us
                    detection_active = False
                    detection_data = None  
                    consecutive_detections = 0
                    
                    # Tell Arduino to start moving forward
                    send_command(ser, "APPROACH")
                    wait_for_response(ser)
                    
                    # On to the next phase
                    current_state = State.APPROACH
                else:
                    # Need a bit more adjustment
                    print(f"Almost there - turning {angle:.1f} degrees for final alignment")
                    send_command(ser, f"TURN:{angle}")
                    wait_for_response(ser)
                    
                    # Give time for the turn to complete
                    time.sleep(3.0)
            else:
                # Lost sight of our target
                print(f"Lost sight of {current_target} during fine alignment - back to search")
                current_state = State.SEARCH
        
        elif current_state == State.APPROACH:
            # Arduino handles the approach using ultrasonic sensor
            # We just monitor for completion messages
            
            # Make absolutely sure detection is disabled during approach
            if current_target == Target.BIN and not aruco_detection_locked:
                print("Making sure ArUco detection is off during approach")
                aruco_detection_locked = True
                detection_active = False
                detection_data = None
            
            # Safety timeout - don't drive forever
            if not hasattr(control_thread, "approach_start_time"):
                control_thread.approach_start_time = time.time()
                print("Starting approach timer")
            
            if time.time() - control_thread.approach_start_time > 120.0:
                print("Safety timeout after 2 minutes of approach - stopping and realigning")
                send_command(ser, "STOP")
                wait_for_response(ser)
                current_state = State.ALIGN_FINE
                detection_active = True  # Turn detection back on for realignment
                
                delattr(control_thread, "approach_start_time")
            
            # Periodically check distance
            if not hasattr(control_thread, "last_distance_check"):
                control_thread.last_distance_check = 0
                
            if time.time() - control_thread.last_distance_check > 1.0:
                send_command(ser, "DISTANCE")
                wait_for_response(ser)
                control_thread.last_distance_check = time.time()
        
        elif current_state == State.ACTION:
            # Arduino is doing the pickup or dropoff
            # We just chill and wait for confirmation
            time.sleep(0.5)  # No need to check super frequently
        
        elif current_state == State.TRANSITION:
            # Handled by Arduino message processing above
            time.sleep(0.5)
        
        elif current_state == State.COMPLETE:
            # All done!
            if not hasattr(control_thread, "complete_message_sent"):
                print("MISSION ACCOMPLISHED!")
                control_thread.complete_message_sent = True
            time.sleep(0.5)
        
        time.sleep(0.1)  # Small delay to prevent hogging the CPU

# Main function
def main():
    global target_aruco_id, AUTO_MODE
    
    print("Starting Autonomous Robotic Waste Sorting System")
    
    # Initialize global variables
    target_aruco_id = 0  # Default to ArUco ID 0
    AUTO_MODE = False
    
    # Initialize components
    picam2 = initialize_camera()
    model = initialize_yolo()
    aruco_dict, aruco_params = initialize_aruco()
    ser = initialize_serial()
    
    # Make serial connection accessible to other functions
    main.serial_connection = ser
    
    # Start threads
    video_thread = Thread(target=video_processing_thread, args=(picam2, model, aruco_dict, aruco_params))
    video_thread.daemon = True
    video_thread.start()
    
    control_thread_instance = Thread(target=control_thread, args=(ser,))
    control_thread_instance.daemon = True
    control_thread_instance.start()
    
    print("System initialized and running. Press Ctrl+C to exit.")
    print("Keyboard shortcuts in camera window:")
    print("  q - Quit program")
    print("  a - Enter AUTO mode")
    print("  o - Enter MANUAL mode")
    print("  1 - Set detected object class to METAL")
    print("  2 - Set detected object class to PLASTIC")
    print("  3 - Set detected object class to PAPER")
    print("  s - Skip to next state")
    
    # Main loop - keep program running and handle keyboard interrupt
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Exiting...")
        cv2.destroyAllWindows()
        picam2.close()

if __name__ == "__main__":
    main() 

