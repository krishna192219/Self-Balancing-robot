# Self-Balancing-robot
A two wheeled self balancing robot build using arduiono and mpu6050 


<p align="center">
  <img 
    src="https://github.com/user-attachments/assets/b41f6a55-211b-4595-87ec-8bacf416cde8"
    alt="Self-Balancing Robot"
    width="500"
  />
</p>



This project is a two-wheel self-balancing robot designed to maintain an upright position using real-time feedback control. The robot is based on the principle of an inverted pendulum, an inherently unstable system in which the wheels must continuously move beneath the robot's center of mass to prevent it from falling. When an external disturbance pushes the robot away from its balance point, the controller detects the change in tilt and drives the motors in the required direction to recover and return the robot to its upright position.

## 🎥 Demonstration

<img width="378" height="550" alt="WhatsAppVideo2026-07-16at23 49 05-ezgif com-optimize" src="https://github.com/user-attachments/assets/34170f84-581f-4859-abc5-0de48bb81556" />



The robot maintains its upright position using real-time tilt estimation
and PID feedback control. When an external disturbance is applied, the
controller drives the wheels to recover the robot and return it toward
its balance point.
## Hardware and Mechanical Design

### Arduino Nano — Main Controller
<img width="400" alt="WhatsApp Image 2026-07-17 at 01 14 41" src="https://github.com/user-attachments/assets/194d2e41-c09f-4828-8120-2461c9227fc6" />


  
The Arduino Nano acts as the main controller of the robot. It continuously
reads data from the IMU, estimates the tilt angle, executes the PID control
algorithm, and generates PWM signals for the motor driver.

I chose the Nano because its compact form factor fits well within the robot's
frame while still providing the required I/O, PWM outputs, and processing
capability for the control loop.

---

### MPU6050 — Inertial Measurement Unit
<img width="411" height="390" alt="image" src="https://github.com/user-attachments/assets/8e6dc94c-f1ad-4244-bc83-0e1f6935ab34" />

The MPU6050 is a 6-DOF IMU containing a 3-axis accelerometer and a 3-axis
gyroscope. It is used to estimate the forward and backward tilt of the robot.

The gyroscope provides fast angular-rate measurements, while the accelerometer
provides an absolute reference for orientation. Since the gyroscope is affected
by long-term drift and the accelerometer is sensitive to vibration and linear
acceleration, I combine both measurements using a complementary filter.

For the physical mounting orientation used in this robot, the pitch estimation
uses the AY accelerometer axis and GX gyroscope axis. 
During the development of the robot, I encountered compatibility issues with the available MPU6050 libraries, which prevented reliable communication with the IMU. Instead of relying on a high-level library, I implemented direct I²C register-level communication between the Arduino Nano and the IMU.

The sensor is initialized by writing directly to its configuration registers, including the power management, digital low-pass filter, gyroscope range, and accelerometer range registers. The raw accelerometer and gyroscope measurements are then read directly from the sensor's data registers over I²C.

This approach gave me greater control over the sensor configuration and also helped me understand how the IMU communicates at a lower level rather than treating it as a black box. The raw AY, AZ, and GX measurements required for balancing are extracted directly and used for tilt estimation. At startup, the robot also collects multiple stationary samples to calculate the gyroscope bias and accelerometer mounting offset, reducing the effect of sensor bias and small mechanical misalignment.

---

### TB6612FNG — Motor Driver
<img width="401" alt="WhatsApp Image 2026-07-16 at 23 49 08" src="https://github.com/user-attachments/assets/4f955aa9-e8a2-40d5-a57d-7eaa33abf27f" />

The two N20 motors are controlled using a TB6612FNG dual H-bridge motor driver.

I selected the TB6612FNG because it is compact, efficient, supports independent
bidirectional control of two DC motors, and responds well to high-frequency PWM
signals. These characteristics are important for a balancing robot, where the
motors must continuously make rapid changes in speed and direction.

---

### N20 Geared DC Motors
<img width="401" alt="WhatsApp Image 2026-07-16 at 23 49 08(1)" src="https://github.com/user-attachments/assets/95245004-bab6-42d4-9ac6-96a67b62a680" />

The robot uses two 400 RPM N20 geared DC motors.

A self-balancing robot requires both sufficient torque and fast wheel response.
The geared motors provide enough torque to accelerate the robot while their
relatively high output speed allows the controller to quickly move the wheels
beneath the robot's center of mass when it begins to fall. They are mounted firmly 
with the help of n20 motors bracket.And they use 43mm rubber wheels which provides 
decent grip when balancing.


During development, I also experimentally determined the minimum PWM required
to overcome the static friction of the motors and gearbox. This value is used
for motor deadzone compensation in the firmware.

---

## Power System

<p align="center">
  <img 
    src="https://github.com/user-attachments/assets/c18b1024-1a91-4133-a499-5683333cf471"
    alt="Self-Balancing Robot Side View"
    width="45%"
  />
  &nbsp;&nbsp;
  <img 
    src="https://github.com/user-attachments/assets/c0e70485-a4ed-43db-96e0-b3d8585d3f1e"
    alt="Self-Balancing Robot Front View"
    width="45%"
  />
</p>


The robot uses two separate power sources for the control electronics and the motors. This separation helps prevent the high current demand and electrical noise generated by the motors from directly affecting the Arduino and sensor electronics.

A **12 V, 1000 mAh, 30C LiPo battery** is used as the main motor power source and supplies the motor side of the TB6612FNG motor driver. The high discharge capability of the LiPo allows it to provide the current required during rapid acceleration, direction changes, and recovery from external disturbances.

A separate **9 V battery** powers the Arduino Nano through its regulated input. The Arduino then powers the low-voltage control electronics, including the IMU. Using a dedicated supply for the controller provides some isolation between the motor power system and the sensitive control electronics.

Both power systems share a **common ground**, which is necessary so that the PWM and direction signals generated by the Arduino have the same voltage reference as the TB6612FNG motor driver.

---

XL6009 DC-DC Boost Converter

<img width="400"  alt="image" src="https://github.com/user-attachments/assets/c0c5bb67-1be6-4f75-a7d2-d30b9dc12e6c" />

An XL6009 adjustable DC-DC boost converter is used in the motor power system to provide a more consistent supply voltage as the battery discharges. The converter is adjusted to provide approximately 12 V to the motor driver, helping reduce changes in motor performance caused by the gradual decrease in battery voltage during operation.

Maintaining a more consistent motor supply is particularly useful for a self-balancing robot because the response of the motors directly affects the behavior of the PID controller. Significant changes in motor speed and torque as the battery discharges can alter the dynamics of the system and affect the tuning of the controller.

## Mechanical Design
<img width="565" height="516" alt="image" src="https://github.com/user-attachments/assets/2a9bb3c4-9e6f-4daf-8b9f-107956c222e7" />


The robot chassis was designed around the inverted-pendulum principle. Two
3mm acrylic sheets having width 9cm length 21cm and they both are placed at a distance of 14cm
and are connected using threaded metal
bolt of 16cm, nuts, and spacers.

The motors and wheels are mounted at the bottom of the structure, while the
battery and control electronics are positioned higher on the chassis this keeps
the center of mass towards the top which helps the robot to fall slower and have 
enough time to react.
The vertical structure provides space for mounting and experimenting with different
component positions during development.

The prototype uses a modular construction approach rather than a permanently
fixed enclosure. This makes the electronics easily accessible and allows the
position of individual components to be adjusted while testing the effect of
weight distribution and center of mass on balancing performance.

---


### Hardware Architecture
## Wiring Schematic


<img width="557" height="561" alt="image" src="https://github.com/user-attachments/assets/fec6479e-3aa0-45b9-b9eb-b18916f0fa20" />




The complete wiring schematic for the self-balancing robot was designed using EasyEDA. The schematic provides a clear overview of the electrical connections between the Arduino Nano, IMU, TB6612FNG motor driver, N20 geared motors, and the power system.

The control electronics and motors use separate power sources. The Arduino Nano and IMU are powered independently from the motor power system, while the 12 V LiPo battery supplies the motors through the XL6009 DC-DC converter and TB6612FNG motor driver. All components share a common ground to provide a common reference for the control signals.

The schematic also shows the I²C connection between the Arduino Nano and the IMU, along with the PWM and direction-control connections used to interface the Arduino with the motor driver.


The overall hardware flow is:

    MPU6050
        |
        v
    Arduino Nano
        |
    PID Controller
        |
        v
    TB6612FNG
       / \
      v   v
    Left  Right
    Motor Motor


## Demonstration



The robot can maintain its upright position and react to external
disturbances by moving its wheels beneath its center of mass.
