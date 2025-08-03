****Chassis and Structural Layout****  
  
The foundation of the robot is a **metal base plate**, which provides mechanical strength and rigidity to support all mounted subsystems. The front section of this base is equipped with the** Steering System**. This steering assembly is actuated by a **servo motor**, which precisely controls the angular orientation of the front wheels. This servo-based control enables accurate navigation and cornering capabilities, critical for path-following algorithms and obstacle avoidance routines.  
  
A key design consideration in this structural setup is the robot’s **minimal ground clearance**, achieved through a **low-profile chassis configuration inspired by Formula 1 car engineering**. This design principle, drawn from the high-performance world of motorsport, enhances **aerodynamic downforce**, which in turn improves **wheel-to-ground contact** during motion. As a result, the robot benefits from significantly **increased stability**, particularly at higher speeds or during sharp directional transitions.  
  
****Drive Mechanism****  
  
Mounted directly behind the steering system is a **motor**, responsible for powering the drivetrain. This motor is mechanically linked to a **differential gearbox**. The gearbox is designed to distribute torque between the two rear wheels, allowing them to rotate at different speeds when the robot turns. This not only reduces wheel slip but also improves turning efficiency and reduces mechanical strain on the drivetrain.  
  
****Battery Placement and Power Routing****  
  
Towards the rear corners and center of the base plate, four **metallic spacers** are mounted vertically, forming a platform to support a secondary **small metal plate**. Beneath this plate, positioned such that it faces the motor, is the** 1000mAh Battery**. This configuration keeps the battery protected and thermally isolated from heat-generating components.  
  
The **wiring from the battery** is carefully routed to the upper layer of the chassis, where it interfaces with the **power distribution board**. This board is responsible for efficiently regulating and distributing electrical power to various subsystems of the robot—including the **Raspberry Pi 5**, **motors**, and other peripheral components.  
  
****Sensor Configuration****  
  
On the same intermediate metal plate, three **VL53L0X laser sensors** are strategically placed. One sensor is aligned to face directly forward, while the other two are oriented at 45° angles to the front-left and front-right. This three sensor configuration enables the robot to detect obstacles and measure distances across a wide field of view.  
  
****Control Unit and Final Layer****  
  
On the topmost platform—secured by an additional set of four spacers above the intermediate plate—rests the final electronic control layer. This layer houses the **Raspberry Pi 5**, the central processing unit responsible for decision-making, motor control, and sensor integration. Alongside the Pi, the **XL4015 Buck Converter** ensures safe voltage input, and a **5MP Raspberry Pi Camera Module** is installed to provide real-time visual input for obstacle and color detection.  
  
****Pictures****  
  
1. First Layer  
![FirstLayer.jpg](Attachments/FirstLayer.jpg)  
2. Second Layer  
![SecondLayer.jpg](Attachments/SecondLayer.jpg)  
3. Third Layer  
![ThirdLayer.jpg](Attachments/ThirdLayer.jpg)  
4. Miscellaneous  
![Miscellaneous_1.jpg](Attachments/Miscellaneous_1.jpg)  
  
![Miscellaneous_2.jpg](Attachments/Miscellaneous_2.jpg)
