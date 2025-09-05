## Team Members  
•⁠ Aashrith Chittamuru \
•⁠ Daksh Singh  \
•⁠ Vatsal Abrol 

**Coach**: \
•⁠ Mr.Kailash *(Roboprenr – Bangalore, India)*  

---

## Introduction
The **WRO2025_Huayra** project is a submission for the World Robot Olympiad (WRO) 2025 under the Future Engineers category. This initiative challenges participants to design, build, and program a self-driving vehicle that can navigate a dynamic environment autonomously. The project integrates mechanical design, embedded systems, and software engineering to create a robust autonomous platform. \
This report outlines the technical architecture, design methodology, software implementation, and theoretical underpinnings of the vehicle developed by Team Huayra.

---

## Project / Bot Description 
Our robot features a **rigid metal chassis reinforced with precision 3D-printed components**, ensuring durability. At its core, a **Raspberry Pi 5** orchestrates real-time processing for **laser sensors, a camera module, and a servo-driven steering system**, enabling highly accurate navigation and control. Its **low ground clearance, inspired by Formula 1 engineering principles, enhances stability and downforce**, ensuring superior performance during high-speed maneuvers. Designed with precision and reliability this platform is built to excel in the **WRO Future Engineers competition**.

---

## Table of Contents 
•⁠ **models**: contains .stl design files of all the 3d printed parts \
•⁠ **others**: contains additional documentation, such as Raspberry Pi setup and terminal usage guides, to help prepare the vehicle for the competition. \
•⁠ **schemes**: contains all the schematic diagrams and illustrates the functioning of the components used in the project \
•⁠ **src**: contains code for all components which were programmed for this competition. It also contains the final code for the competition \
•⁠ **videos**: includes two videos of the bot functioning \
•⁠ **vehicle-photos**: contains 6 photos of the bot (front, back, left, right, top, bottom) \
•⁠ **team-photos**: contains 2 images of our team (1 formal and 1 funny)

---

## Hardware Components

| Component                 | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| Arduino Uno               | Microcontroller for low-level sensor interfacing and motor control          |
| Raspberry Pi              | High-level processing unit for decision-making and image processing         |
| Ultrasonic Sensors        | Distance measurement for obstacle detection                                 |
| Servo Motors              | Steering mechanism                                                          |
| DC Motors with Encoders   | Drive system with feedback for speed and position                           |
| Li-ion Battery Pack       | Power source with voltage regulation                                        |


## Software Stack

•⁠  ⁠*Arduino IDE*: Used to program the microcontroller \
•⁠  ⁠*Python (on Raspberry Pi)*: Handles image recognition, path planning, and serial communication \
•⁠  ⁠*OpenCV*: For visual input processing and object detection

---

## Serial Communication Protocol

The ⁠ SerialCommunicationArduino.ino ⁠ file implements a custom protocol to facilitate reliable data exchange between the Arduino and Raspberry Pi. Key features include:

•⁠  ⁠Baud rate synchronization at 9600 bps \
•⁠  ⁠Real-time sensor data transmission

This protocol ensures deterministic behavior and low-latency control, critical for autonomous navigation.

---

## Mechanical Design

The chassis and structural components were designed with our critical thinking abilities using metal plates and CAD software and fabricated using 3D printing. The car's design was inspired from the old F1 cars from the early 2000's where the real wheels were bigger size than the front wheels as the rear drving system was built higher in the rear of the car than the front. STL files in the repository’s ⁠ models ⁠ folder provide the geometry for:

•⁠  ⁠Wheel mounts \
•⁠  ⁠Sensor brackets \
•⁠  ⁠Battery housing \
•⁠  ⁠Modular frame connectors

The design prioritizes weight distribution, accessibility, and modularity for quick maintenance and upgrades.

---

## Circuit Schematics

The ⁠ schemes ⁠ folder contains detailed circuit diagrams illustrating the integration of sensors, actuators, and controllers. These schematics follow best practices in embedded system design:

•⁠  ⁠Insulating the metal base plates with tape to avoid short-circuiting \
•⁠  ⁠Integration between arduino and Raspberry Pi 5 \
•⁠  ⁠Fuse protection for motor drivers

---

## Control Algorithms

•⁠  ⁠**Open Challenge**: Ultrasonic sensors detect the walls (boundaries) and the arduino adjusts motor speed as and when the car is turning or going straight. \
•⁠  ⁠**Obstacle Avoidance**: Camera reaches a specific area limit of the color due to which it turns. \
•⁠  ⁠**Parking Algorithm**: Uses Ultrasonic sensor feedback and servo positioning to execute parallel parking.

---

## Conclusion

The WRO2025_Huayra project reflects our team’s dedication to combining mechanical design, electronics, and advanced software algorithms into a single autonomous vehicle capable of tackling the dynamic challenges of the WRO Future Engineers competition. By leveraging the strengths of both the Arduino Uno and Raspberry Pi 5, we achieved a balance between low-level real-time control and high-level decision-making. Our focus on rigid mechanical design, robust sensor integration, and modular architecture ensures that the vehicle is not only competition-ready but also adaptable for future improvements.
