# Chassis and Structural Layout 
  
The foundation of the robot is a **metal base plate**, which provides mechanical strength and rigidity to support all mounted subsystems. The front section of this base is equipped with the **Steering System**. This steering assembly is actuated by a **servo motor**, which precisely controls the angular orientation of the front wheels. This servo-based control enables accurate navigation and cornering capabilities, critical for path-following algorithms and obstacle avoidance routines.  
  
A key design consideration in this structural setup is the robot’s **minimal ground clearance**, achieved through a **low-profile chassis configuration inspired by Formula 1 car engineering**. This design principle, drawn from the high-performance world of motorsport, enhances **aerodynamic downforce**, which in turn improves **wheel-to-ground contact** during motion. As a result, the robot benefits from significantly **increased stability**, particularly at higher speeds or during sharp directional transitions.  
  
# Drive Mechanism 
  
Positioned **directly behind the steering assembly**, the robot features a high-efficiency **motor**, which serves as the **primary source** of **mechanical power** for the drivetrain. The output shaft of the motor is **mechanically coupled to a differential gearbox**, a critical component of the drive system. The **differential gearbox** performs the essential function of **splitting torque** between the two rear wheels. It allows each wheel to rotate at a different speed while still receiving power from the same motor. This functionality becomes especially important during turning maneuvers, where the inner and outer wheels naturally travel different distances.  
  
By enabling differential wheel speeds, the gearbox:  
	•	**Reduces unwanted wheel slippage**, preserving traction and improving handling.  
	•	**Enhances turning efficiency**, allowing the robot to navigate curves smoothly and with greater precision.  
	•	**Minimizes mechanical stress** on the drivetrain components, thereby improving long-term durability and energy efficiency. 
  
# Battery Placement and Power Routing
  
Towards the rear corners and center of the base plate, four **metallic spacers** are mounted vertically, forming a platform to support a secondary **small metal plate**. Beneath this plate, positioned such that it faces the motor, is the **1000mAh Battery**. This configuration keeps the battery protected and thermally isolated from heat-generating components.  
  
The **wiring from the battery** is carefully routed to the upper layer of the chassis, where it interfaces with the **power distribution board**. This board is responsible for efficiently regulating and distributing electrical power to various subsystems of the robot—including the **Raspberry Pi 5**, **motors**, and other peripheral components.  
  
# Sensor Configuration  
  
Mounted on the intermediate **metal plate**, the robot features a strategically arranged **three-sensor array** composed of **VL53L0X TOF (Time-of-Flight) laser distance sensors**. These compact, high-precision sensors are positioned to optimize the robot’s spatial awareness and real-time environmental perception.  
  
One sensor is oriented to face **directly forward**, providing accurate distance measurements along the primary axis of movement. The remaining two sensors are mounted at **45° angles** to the **front-left** and **front-right**, forming a wide-angle, triangular sensing pattern. This arrangement significantly expands the robot’s **field of view**, enabling it to monitor a broader area in front of it. The triangulated configuration allows the robot to **simultaneously detect obstacles**, measure varying distances, and estimate angles of approaching objects.  
  
# Control Unit and Final Layer  
  
The **topmost platform** of the robot is mounted securely above the intermediate plate using **four precision-aligned metal spacers**, forming the final and most critical layer—the **electronic control and processing unit**. This elevated placement not only provides physical isolation from the lower layer but also facilitates easy access for maintenance and debugging. At the heart of this control layer lies the **Raspberry Pi 5**, which functions as the **central processing unit (CPU)** of the robot. It is responsible for executing all core decision-making algorithms.  
  
To ensure that the Raspberry Pi and its peripherals receive a stable and safe voltage, the control layer includes an **XL4015 DC-DC Buck Converter Module**. This component steps down the 7.4V input from the LiPo battery to the 5V required by the Raspberry Pi, while maintaining a high efficiency and current capacity. Its inclusion safeguards the sensitive electronics from voltage spikes and under-voltage conditions, both of which could otherwise result in operational instability or hardware damage. Also mounted on this platform is a **5MP Raspberry Pi Camera Module**, which provides the robot with real-time **visual input**. This module enables a variety of vision-based functions, such as obstacle and color detection.  
  
# Pictures  
 
1. First Layer  
![FirstLayer.jpg](Attachments/FirstLayer.jpg)  
2. Second Layer  
![SecondLayer.jpg](Attachments/SecondLayer.jpg)  
3. Third Layer  
![ThirdLayer.jpg](Attachments/ThirdLayer.jpg)  
4. Miscellaneous  
![Miscellaneous_1.jpg](Attachments/Miscellaneous_1.jpg)

![Miscellaneous_2.jpg](Attachments/Miscellaneous_2.jpg)
