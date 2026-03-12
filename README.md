# PuzzleBot Nonholonomic Control

ROS2-based control framework for a differential-drive robot with 5 controllers 
(PID, SMC, ISMC, CTC, Port-Hamiltonian), Gazebo simulation, real hardware 
integration via ESP32 and micro-ROS, and Lyapunov stability analysis.

## Hardware
- ESP32 + L298N + 2x JGA25-370 motors with encoders
- micro-ROS over WiFi UDP

## Credits
Based on the academic work of Ferguson, Donaire, Renton & Middleton (2018).
Original implementation reference: https://github.com/nezih-niegu/pb-j_control

## Video
https://youtu.be/RniDI9pE5dg
