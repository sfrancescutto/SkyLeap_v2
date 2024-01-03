This project was developed as part of the "integrated and mobile systems" course in the master's degree program 
in electronics engineering. The project, supervised by Professor S.Carrato, was developed using tools and components 
provided by Professor S.Carrato and by "Università degli studi di Trieste" laboratories. 

Hardware components:
- FRDM KL25Z, microntroller by NXP
- MPU 9150, 9 axis IMU sensor by Invensense
- 4 brushless motors with associated ESCs
- RF reciver and remote control
- custom-developed shield with led, buzzer, switches and buttons
- Quadcopter drone frame 
- Power generetor

The code is complete however the tuning is not. The following changes and implementations are needed: PID tuning, improvement of 
yaw correction, complementary filter parameters tuning, improvement of magnetometer reading time.