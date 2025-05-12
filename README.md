#Autonomous Waste Sorting Robot

![Robot in action](https://via.placeholder.com/800x400?text=Waste+Sorting+Robot)

## 👋 Hi there!

Welcome to our project repository! We're Jamie and Alex, two engineering students who built this autonomous waste sorting robot for our final year capstone project. We wanted to help solve the global recycling challenge by creating an accessible robotics solution that can identify and sort recyclable materials.

## ✨ What it does

Our robot can:
- Autonomously navigate its environment
- Identify recyclable objects (metal, plastic, paper) using computer vision
- Pick up objects with its robotic arm
- Identify recycling bins using ArUco markers
- Deposit objects in the correct bin based on material type
- Be controlled manually through a Bluetooth app

## 🛠️ How we built it

### Hardware Components:
- Arduino Mega (motor and arm control)
- Raspberry Pi 4 (computer vision and decision making)
- 4-wheel drive chassis with DC motors
- 6-DOF robotic arm with servo motors
- Ultrasonic distance sensor
- Raspberry Pi Camera v2
- 3D printed parts for arm and gripper
- Custom recycling bins with ArUco markers

### Software Architecture:
- **Arduino**: Controls motors, servo arm, reads distance sensor, and manages state machine
- **Raspberry Pi**: Runs computer vision (YOLOv8 & ArUco detection), calculates positioning, and sends commands to Arduino
- **Communication**: UART serial between Pi and Arduino
- **Object Detection**: Custom-trained YOLOv8 model for detecting recyclable objects
- **Bin Detection**: ArUco marker detection for identifying different recycling bins

## 🔬 The Development Process

This project has been a crazy journey for both of us! We spent countless late nights in the lab tweaking the arm movements, fine-tuning the object detection model, and figuring out why our robot kept turning the wrong way (spoiler: we had the motor wires mixed up).

One of our biggest challenges was getting the arm to pick up objects reliably. We ended up implementing a two-stage approach where the arm positions itself above the object first, then gently lowers down to make the pickup smoother. The gripper closure is also done gradually to avoid knocking over the object.

Training our own object detection model was another adventure. We collected over 500 images of various recyclable items in different lighting conditions and orientations. After training the model, we converted it to NCNN format to run efficiently on the Raspberry Pi.

## 🚀 How to Run It

### Prerequisites
- Arduino IDE
- Python 3.7+
- OpenCV with ArUco module
- Ultralytics YOLO
- Picamera2 library
- Assembled robot with all hardware components

### Setup Instructions
1. Upload the `arduino_code.ino` to your Arduino Mega
2. Install required Python packages on the Raspberry Pi:
   ```
   pip install opencv-python numpy ultralytics picamera2 imutils pyserial
   ```
3. Copy the `rpi_code.py` and YOLOv8 model to your Raspberry Pi
4. Run the Python script on the Pi:
   ```
   python rpi_code.py
   ```
5. The robot can be started in auto mode either through the Python interface or via Bluetooth app

## 🌟 Future Improvements

We have several ideas for future improvements:
- Add a larger capacity battery for longer operation
- Improve the object detection model with more training data
- Implement a more sophisticated path planning algorithm
- Create a web interface for remote monitoring
- Add sound feedback for more intuitive interaction

## 📝 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

We'd like to thank:
- Our amazing project advisor, Dr. Johnson, for the guidance and letting us stay in the lab after hours
- The university's engineering department for providing equipment and funding
- The open source community for the libraries and tools that made this possible
- Our fellow students who helped test the robot and gave valuable feedback

---

*Built with ❤️ by Jamie and Alex*

*University of Engineering Excellence, Class of 2023* 
