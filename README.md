## OpticalFlowSlider 

### Overview
is a Python and Docker-based project that explores real-time optical flow for motion control in Autonomous Underwater Vehicles (AUVs). The project leverages an Intel RealSense D435i camera combined with inertial sensors (both onboard and an external InertialSense RUG-3) to estimate motion and improve vehicle orientation and control in dynamic underwater environments.

### Objective:
Enhance AUV navigation by fusing optical flow data with inertial measurements and motor feedback. This sensor fusion (via a Kalman filter) aims to provide accurate and robust state estimates for improved control.

### Test-Bench:
A custom slider platform equipped with:
- Camera: Intel RealSense D435i (RGB video & IMU)
- External IMU: InertialSense RUG-3
- Actuation: MX-106 Dynamixel motor with gearing and a GT2 pulley system
- The slider simulates controlled linear motion to validate sensor fusion and optical flow algorithms.

### Key Modules:
- Camera & IMU Nodes: Stream video and IMU data via ROS 2 topics.
- Optical Flow: Offline and real-time processing to estimate motion.
- Data Fusion: Kalman filter integration combining optical flow, IMU, and motor data.
- Controller: Generates motor commands based on the fused state estimate.
- ROS 2 Communication: Ensures seamless inter-module data exchange.

### Features

- Real-Time Optical Flow: Test and optimize both classical and AI-based methods (e.g., LiteFlowNet).
- Sensor Fusion: Combines optical flow with inertial and motor feedback for robust state estimation.
- Modular ROS 2 Architecture: Simplifies integration, testing, and future expansion.
- Dockerized Environment: Provides a consistent setup for development and deployment.

![alt text](Readme/image.png)


![alt text](<Readme/image copy.png>)
