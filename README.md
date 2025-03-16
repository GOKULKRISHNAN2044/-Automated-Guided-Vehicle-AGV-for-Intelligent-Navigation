#  Automated Guided Vehicle (AGV) for Intelligent Navigation

## 📌 Overview

This project implements an **Automated Guided Vehicle (AGV)** equipped with intelligent navigation capabilities. The AGV autonomously navigates an environment, avoids obstacles, and follows an optimal path using advanced sensors and AI-based decision-making.

## 🎯 Features

- **Autonomous Navigation**: Uses LiDAR, ultrasonic, and infrared sensors for self-navigation.
- **Obstacle Avoidance**: Detects obstacles and reroutes using sensor data.
- **Path Planning**: Implements A* and Dijkstra’s algorithm for optimal movement.
- **Real-time Monitoring**: Web-based dashboard for live tracking and remote control.
- **Machine Learning Integration**: AI-based decision-making for dynamic environments.

---

## 🛠️ Hardware Components

### 🎛️ **Microcontroller / Processor**
| Component | Description |
|-----------|------------|
| **Raspberry Pi 4 / Jetson Nano** | Main processing unit for AI and navigation. |
| **Arduino Mega 2560** | Handles motor control and sensor input. |

### 🔌 **Motors and Motion Control**
| Component | Description |
|-----------|------------|
| **DC Motors with Encoders** | Enables precise speed control and movement. |
| **Motor Driver (L298N / TB6612FNG)** | Controls motor speed and direction. |

### 🛑 **Sensors**
| Sensor | Purpose |
|--------|---------|
| **LiDAR (RPLiDAR A1/A2)** | Maps surroundings and detects obstacles. |
| **Ultrasonic Sensors (HC-SR04)** | Measures distance from objects. |
| **Infrared (IR) Sensors** | Detects edges and line-following paths. |
| **IMU (MPU6050 / BNO055)** | Measures orientation and movement. |

### 🔋 **Power Supply**
| Component | Description |
|-----------|------------|
| **Li-ion Battery Pack (12V 5000mAh)** | Provides power to motors and controllers. |
| **Voltage Regulator (LM2596)** | Ensures stable voltage supply. |

### 📡 **Communication & Interfaces**
| Component | Description |
|-----------|------------|
| **Wi-Fi Module (ESP8266 / ESP32)** | Enables remote monitoring and control. |
| **Camera Module (Raspberry Pi Camera v2)** | (Optional) Vision-based navigation. |

---

## 💻 Software Requirements

### 🔹 **Programming Languages**
- **Python** – Main programming language.
- **C++** – For low-level motor and sensor control.

### 🔹 **Frameworks & Libraries**
- **Robot Operating System (ROS)** – Communication between hardware and software components.
- **OpenCV** – For image processing and vision-based navigation.
- **NumPy, Pandas, SciPy** – For numerical computations and sensor data processing.
- **Flask** – For the web-based interface.
- **TensorFlow / PyTorch** – If AI-based navigation is implemented.

---

## 🚀 Installation & Setup

### 1️⃣ Clone the Repository:
bash:
git clone https://github.com/GOKULKRISHNAN2044/Automated-Guided-Vehicle-AGV-for-Intelligent-Navigation.git
cd Automated-Guided-Vehicle-AGV-for-Intelligent-Navigation
###2️⃣ Set Up Virtual Environment:
bash
Copy
Edit
python -m venv env
source env/bin/activate  # On Windows: env\Scripts\activate
###3️⃣ Install Dependencies:
bash
Copy
Edit
pip install -r requirements.txt
###4️⃣ ROS Installation (If using ROS)
Install ROS Noetic (Ubuntu 20.04) or ROS2 Foxy (for modern applications).
Set up the workspace and build the AGV navigation package.
###5️⃣ Hardware Setup
Connect the motors, sensors, and camera according to the circuit diagram.
Power up the AGV and ensure all connections are secure.
###6️⃣ Run the Application
bash
Copy
Edit
roslaunch agv_navigation main.launch

The AGV will start mapping its environment and move autonomously.

Use the web interface for manual control and real-time monitoring.
##🎮 Usage:

Autonomous Mode: The AGV moves based on pre-set navigation algorithms.
Manual Mode: Users can control the AGV remotely via a dashboard.
Obstacle Detection: The AGV stops or reroutes upon detecting obstacles.

##📊 Data Logging & Analysis:

Logs all sensor readings in CSV format for further analysis.
Visualization of movement paths using Matplotlib & OpenCV.
🤝 Contribution Guidelines
Fork the repository.
Create a new branch (feature-xyz).
Commit changes and submit a pull request.


