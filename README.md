# XTreads: An Omnidirectional X-Configuration Treaded Vehicle

This repository contains the complete design, documentation, and code for **XTreads**, an experimental omnidirectional ground vehicle. [cite_start]The project's core innovation is a unique chassis with four independently actuated treaded systems arranged in an "X" shape[cite: 14, 27]. [cite_start]This configuration is designed to provide seamless $360^{\circ}$ movement without needing to yaw (turn)[cite: 14], combining the all-terrain capability of tracks with the maneuverability of an omni-wheel.

!(https://i.imgur.com/your-image-link.png) ## Motivation

Traditional robotic platforms face a trade-off:
* [cite_start]**Wheeled Omni-Systems (e.g., Mecanum):** Offer high maneuverability on flat surfaces but perform poorly on rough, loose, or uneven terrain[cite: 24].
* [cite_start]**Conventional Treads:** Provide excellent all-terrain performance and load-bearing capacity [cite: 22] [cite_start]but lack true omnidirectional motion, relying on inefficient skid-steering to turn[cite: 22].

[cite_start]XTreads was conceptualized to bridge this gap, creating a robust platform that offers high mobility in confined spaces [cite: 15] [cite_start]and can support heavy-weight applications where other omni-systems fail[cite: 21, 25].

## Key Mechanical Features

[cite_start]The vehicle is built from four identical, independently controlled tread modules[cite: 27].

* [cite_start]**X-Configuration:** Four tread assemblies are arranged in an 'X' pattern[cite: 27], allowing for resultant forces to be generated in any direction (Forward, Backward, Left, Right, and all diagonals).
* [cite_start]**Dynamic Tread Lifting:** To minimize drag and friction during diagonal or lateral movements, redundant treads are actively lifted off the ground[cite: 122]. [cite_start]This is achieved using a **servo motor** [cite: 58] [cite_start]that pulls a **high-tensile nylon wire** [cite: 59][cite_start], which is connected to the tread's angling linkage[cite: 48, 59]. [cite_start]The system relies on gravity and a spring suspension for downward motion[cite: 60].
* [cite_start]**Slider-Spring Suspension:** Each tread module incorporates a slider-spring mechanism [cite: 53][cite_start], allowing it to absorb shocks and maintain ground contact when maneuvering over small obstacles[cite: 54, 56].
* [cite_start]**Chain Tensioning System:** A screw-based mechanism is integrated into each module [cite: 44][cite_start], allowing the driven sprocket's position to be adjusted to ensure optimal chain tension[cite: 40, 41, 45].
* [cite_start]**Drive Assembly:** Each tread is powered by a **NEMA-17 stepper motor** [cite: 28] [cite_start]connected to a drive sprocket via a chain [cite: 32][cite_start], ensuring precise motion control[cite: 28].

## Hardware & Control Architecture

The system uses a two-level control architecture to manage high-level planning and low-level real-time actuation.

### Electronics
* [cite_start]**High-Level Controller (Brain):** A **Raspberry Pi 5** serves as the central brain[cite: 89]. [cite_start]It runs the user-facing GUI [cite: 94][cite_start], handles high-level computation [cite: 90][cite_start], and sends directional commands to the low-level controller[cite: 91].
* [cite_start]**Low-Level Controller (Actuation):** An **Arduino Mega 2560** paired with a **RAMPS 1.4 shield** acts as the low-level controller[cite: 74, 75]. [cite_start]It receives simple serial commands (e.g., 'F', 'B') [cite: 130] [cite_start]from the RPi [cite: 79] [cite_start]and handles the real-time step and direction pulsing for the stepper motors[cite: 79].
* [cite_start]**Motor Drivers:** **DRV8825** drivers, slotted into the RAMPS board, manage the power for the four NEMA-17 stepper motors[cite: 77, 123].
* [cite_start]**Sensors:** An **MPU6050 IMU** (Inertial Measurement Unit) [cite: 125] [cite_start]provides real-time orientation and acceleration data[cite: 97, 125]. [cite_start]This feedback is used by the RPi to stabilize the vehicle [cite: 126] [cite_start]and correct for any deviation from the intended trajectory[cite: 127].
* **Actuators:**
    * [cite_start]**4x NEMA-17 Stepper Motors** for tread propulsion[cite: 82, 108].
    * [cite_start]**4x Servo Motors** for the tread-lifting mechanism [cite: 58, 86][cite_start], controlled by the Raspberry Pi 5[cite: 87].

### Power System
The power architecture is designed for stability by isolating the sensitive computational hardware.
* [cite_start]**Main Power:** A **4S LiPo Battery** (14.8V) powers the system[cite: 68, 133].
* **Voltage Regulation:**
    * [cite_start]A **12V Buck Converter** steps down voltage for the RAMPS 1.4 board and motors[cite: 70].
    * [cite_start]A **5V Buck Converter** steps down voltage for the servo motors[cite: 71].
* [cite_start]**Isolated RPi Power:** A separate **20,000 mAh power bank** is dedicated to the Raspberry Pi 5 [cite: 72][cite_start], protecting it from voltage sags or noise caused by the high-current motors[cite: 72].

## Control Logic

1.  [cite_start]The user inputs a direction using **keyboard arrow keys**[cite: 139, 166].
2.  [cite_start]A **Pygame Direction Mapper** [cite: 140] [cite_start]running on the RPi 5 converts this input into one of eight compass directions (N, S, E, W, NE, NW, SE, SW) [cite: 143, 167-169].
3.  [cite_start]The RPi 5 then issues two commands simultaneously[cite: 141, 144, 148]:
    * [cite_start]**Serial Control Path:** It sends an ASCII character (e.g., 'L') [cite: 130] [cite_start]via serial (UART) to the **Arduino Mega**[cite: 129, 149]. [cite_start]The Arduino interprets this [cite: 130] [cite_start]and activates the appropriate **Stepper Motors** via the RAMPS shield[cite: 151, 173].
    * [cite_start]**GPIO PWM Path:** It sends PWM signals directly to the **Tread Servo Actuators**[cite: 144, 150]. [cite_start]A "Diagonal Mode Logic" determines which treads to lift or lower based on the command[cite: 147, 174].
4.  [cite_start]The **IMU** continuously provides feedback to the RPi [cite: 177][cite_start], which can then adjust motor commands to ensure stable movement[cite: 127].

## GUI
A custom GUI built with Pygame runs on the Raspberry Pi and provides:
* [cite_start]An 8-directional indicator for navigation[cite: 199].
* [cite_start]Real-time IMU heading and data logging[cite: 203, 206, 207].
* [cite_start]A serial monitor for debugging communication with the Arduino[cite: 207].
* [cite_start]Controls for servo calibration and angle adjustment[cite: 208].
* [cite_start]A servo mapping interface to define which servos activate for each directional command[cite: 209].

!(https://i.imgur.com/your-gui-image-link.png) ## Future Prospects

[cite_start]While the prototype successfully demonstrated the concept, the following enhancements are planned for future iterations[cite: 241]:
* [cite_start]**Actuators:** Replace NEMA-17 steppers with **high-torque DC motors** [cite: 246] [cite_start]and integrate more robust, **high-torque servos** for the lifting mechanism[cite: 256].
* [cite_start]**Materials:** Transition to a **carbon fiber and aluminum frame** to reduce weight while maintaining strength [cite: 259-260].
* [cite_start]**Electronics:** Discretize the hardware using smaller boards (like ESP32 or compact Arduinos) and independent drivers to reduce the vehicle's overall height and save space[cite: 243, 249].
* [cite_start]**Power:** Shift from a single LiPo battery to **distributed Li-Ion cell arrays** for better weight distribution and reliability[cite: 251].
* [cite_start]**Autonomy:** Integrate **real-time terrain mapping** and develop **autonomous route planning** algorithms[cite: 262].
* [cite_start]**Specialization:** Develop application-focused versions for tasks like load handling, stealth missions, or explosive deployment[cite: 264].

## Team Members
* [cite_start]Hrishikesh Jawale [cite: 3]
* [cite_start]Prathamesh Kawtikwar [cite: 4]
* [cite_start]Harsh Sinha [cite: 5]
* [cite_start]Kaushiki Nanda Duarah [cite: 6]
* [cite_start]Hrishikesh Hiremath [cite: 7]
* [cite_start]Sahil Umale [cite: 8]

## Repository Contents
* `CAD/`: 3D models and mechanical designs.
* `Code/`:
    * `RaspberryPi/`: Python scripts for the Pygame GUI, high-level control logic, and IMU interfacing.
    * `Arduino/`: Arduino Mega sketch for low-level motor control.
* `Documentation/`: Project report and datasheets.
* `Data/`: Experimental test data and performance evaluation.
