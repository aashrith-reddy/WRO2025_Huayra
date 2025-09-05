## Team Members  
- Aashrith Chittamuru
- Daksh Singh 
- Vatsal Abrol 

**Coach**: 
- Mr.Kailash *(Roboprenr – Bangalore, India)*  

---

## Introduction
The **WRO2025_Huayra** project is a submission for the World Robot Olympiad (WRO) 2025 under the Future Engineers category. This initiative challenges participants to design, build, and program a self-driving vehicle that can navigate a dynamic environment autonomously. The project integrates mechanical design, embedded systems, and software engineering to create a robust autonomous platform.
This report outlines the technical architecture, design methodology, software implementation, and theoretical underpinnings of the vehicle developed by Team Huayra.

---

## Project / Bot Description  
Our robot features a **rigid metal chassis reinforced with precision 3D-printed components**, ensuring durability. At its core, a **Raspberry Pi 5** orchestrates real-time processing for **laser sensors, a camera module, and a servo-driven steering system**, enabling highly accurate navigation and control. Its **low ground clearance, inspired by Formula 1 engineering principles, enhances stability and downforce**, ensuring superior performance during high-speed maneuvers. Designed with precision and reliability this platform is built to excel in the **WRO Future Engineers competition**.

---

## Table of Contents  
- **models**: contains .stl design files of all the 3d printed parts
- **others**: contains additional documentation, such as Raspberry Pi setup and terminal usage guides, to help prepare the vehicle for the competition.
- **schemes**: contains all the schematic diagrams and illustrates the functioning of the components used in the project
- **src**: contains code for all components which were programmed for this competition. It also contains the final code for the competition
- **videos**: includes two videos of the bot functioning
- **vehicle-photos**: contains 6 photos of the bot (front, back, left, right, top, bottom)
- **team-photos**: contains 2 images of our team (1 formal and 1 funny)

---

## Objectives

•⁠  ⁠Design and fabricate a compact autonomous vehicle using additive manufacturing and CNC techniques.
•⁠  ⁠Implement reliable serial communication between microcontrollers and single-board computers.
•⁠  ⁠Develop modular control software for navigation, obstacle avoidance, and task execution.
•⁠  ⁠Comply with WRO Future Engineers 2025 rules and evaluation criteria.


---

### 🔧 Hardware Components

| Component                 | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| Arduino Uno               | Microcontroller for low-level sensor interfacing and motor control          |
| Raspberry Pi              | High-level processing unit for decision-making and image processing         |
| Ultrasonic Sensors        | Distance measurement for obstacle detection                                 |
| IR Sensors                | Line-following and edge detection                                           |
| Servo Motors              | Steering mechanism                                                          |
| DC Motors with Encoders   | Drive system with feedback for speed and position                           |
| Li-ion Battery Pack       | Power source with voltage regulation                                        |




