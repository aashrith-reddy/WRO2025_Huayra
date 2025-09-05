## Team Members  
•⁠ Aashrith Chittamuru
•⁠ Daksh Singh 
•⁠ Vatsal Abrol 

**Coach**: 
•⁠ Mr.Kailash *(Roboprenr – Bangalore, India)*  

---

## Introduction
The **WRO2025_Huayra** project is a submission for the World Robot Olympiad (WRO) 2025 under the Future Engineers category. This initiative challenges participants to design, build, and program a self-driving vehicle that can navigate a dynamic environment autonomously. The project integrates mechanical design, embedded systems, and software engineering to create a robust autonomous platform.
This report outlines the technical architecture, design methodology, software implementation, and theoretical underpinnings of the vehicle developed by Team Huayra.

---

## Project / Bot Description  
Our robot features a **rigid metal chassis reinforced with precision 3D-printed components**, ensuring durability. At its core, a **Raspberry Pi 5** orchestrates real-time processing for **laser sensors, a camera module, and a servo-driven steering system**, enabling highly accurate navigation and control. Its **low ground clearance, inspired by Formula 1 engineering principles, enhances stability and downforce**, ensuring superior performance during high-speed maneuvers. Designed with precision and reliability this platform is built to excel in the **WRO Future Engineers competition**.

---

## Table of Contents  
•⁠ **models**: contains .stl design files of all the 3d printed parts
•⁠ **others**: contains additional documentation, such as Raspberry Pi setup and terminal usage guides, to help prepare the vehicle for the competition.
•⁠ **schemes**: contains all the schematic diagrams and illustrates the functioning of the components used in the project
•⁠ **src**: contains code for all components which were programmed for this competition. It also contains the final code for the competition
•⁠ **videos**: includes two videos of the bot functioning
•⁠ **vehicle-photos**: contains 6 photos of the bot (front, back, left, right, top, bottom)
•⁠ **team-photos**: contains 2 images of our team (1 formal and 1 funny)

---

## Objectives

•⁠  ⁠Design and fabricate a compact autonomous vehicle using additive manufacturing and CNC techniques.
•⁠  ⁠Implement reliable serial communication between microcontrollers and single-board computers.
•⁠  ⁠Develop modular control software for navigation, obstacle avoidance, and task execution.
•⁠  ⁠Comply with WRO Future Engineers 2025 rules and evaluation criteria.


---

### Hardware Components

| Component                 | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| Arduino Uno               | Microcontroller for low-level sensor interfacing and motor control          |
| Raspberry Pi              | High-level processing unit for decision-making and image processing         |
| Ultrasonic Sensors        | Distance measurement for obstacle detection                                 |
| IR Sensors                | Line-following and edge detection                                           |
| Servo Motors              | Steering mechanism                                                          |
| DC Motors with Encoders   | Drive system with feedback for speed and position                           |
| Li-ion Battery Pack       | Power source with voltage regulation                                        |


### Software Stack

•⁠  ⁠*Arduino IDE*: Used to program the microcontroller (⁠ SerialCommunicationArduino.ino ⁠)
•⁠  ⁠*Python (on Raspberry Pi)*: Handles image recognition, path planning, and serial communication
•⁠  ⁠*OpenCV*: For visual input processing and object detection

---

## Serial Communication Protocol

The ⁠ SerialCommunicationArduino.ino ⁠ file implements a custom protocol to facilitate reliable data exchange between the Arduino and Raspberry Pi. Key features include:

•⁠  ⁠Baud rate synchronization at 9600 bps
•⁠  ⁠Command parsing using delimiters
•⁠  ⁠Error handling and acknowledgment signals
•⁠  ⁠Real-time sensor data transmission

This protocol ensures deterministic behavior and low-latency control, critical for autonomous navigation.

---

## Mechanical Design

The chassis and structural components were designed with our critical thinking abilities using metal plates and CAD software and fabricated using 3D printing. The car's design was inspired from the old F1 cars from the early 2000's where the real wheels were bigger size than the front wheels as the rear drving system was built higher in the rear of the car than the front. STL files in the repository’s ⁠ models ⁠ folder provide the geometry for:

•⁠  ⁠Wheel mounts
•⁠  ⁠Sensor brackets
•⁠  ⁠Battery housing
•⁠  ⁠Modular frame connectors

The design prioritizes weight distribution, accessibility, and modularity for quick maintenance and upgrades.

---

## Circuit Schematics

The ⁠ schemes ⁠ folder contains detailed circuit diagrams illustrating the integration of sensors, actuators, and controllers. These schematics follow best practices in embedded system design:

•⁠  ⁠Insulating the metal base plates with tape to avoid short-circuiting
•⁠  ⁠Pull-up resistors for digital inputs
•⁠  ⁠Integration between arduino and Raspberry Pi 5
•⁠  ⁠Fuse protection for motor drivers

---

## Control Algorithms

### Navigation Logic

•⁠  ⁠**Wall Following**: Ultrasonic sensors detect the walls (boundaries) and PID control adjusts motor speed as and when the car is turning or going straight.
•⁠  ⁠**Obstacle Avoidance**: Ultrasonic sensors trigger evasive maneuvers using pre-defined routines.
•⁠  ⁠**Parking Algorithm**: Uses Ultrasonic sensor feedback and servo positioning to execute parallel parking.


### Decision-Making

•⁠  ⁠Finite State Machine (FSM) governs task transitions.
•⁠  ⁠Fusion of perception combines Camera and ultrasonic data for robust environmental awareness.
•⁠  ⁠Optional integration of computer vision for traffic sign recognition.

---

## Compliance with WRO Rules

The project adheres to the WRO Future Engineers 2025 guidelines:

•⁠  ⁠Age group eligibility extended to 22 years
  -Vatsal: 16 years old
  -Aashrith: 16 years old
  -Daksh: 16 years old
•⁠  ⁠Vehicle dimensions and weight within specified limits
  -Our car might be the longest one but still well within the boundaries.
•⁠  ⁠Engineering documentation hosted on GitHub as required
  -We ghave created a rather quite attractive GitHub Repositery.

---

## Evaluation Criteria

| Criterion                 | Description                                                                  |
|---------------------------|------------------------------------------------------------------------------|
| Technical Innovation      | Custom serial protocol and modular chassis design                            |
| Engineering Documentation | Comprehensive GitHub repository with code, models, and schematics            |
| Task Execution            | Reliable navigation and obstacle avoidance routines                          |
| Presentation              | Clear articulation of design choices and theoretical foundations             |

---

## Conclusion

The "WRO2025_Huayra" project exemplifies a multidisciplinary approach to autonomous vehicle development. By combining mechanical engineering, embedded systems, and software architecture, the team has created a scalable and competition-ready platform. The repository serves as a transparent and educational resource for future robotics enthusiasts.











