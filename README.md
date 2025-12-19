# Robot Arm v2 (ESP32)

An ESP32-based 6-DOF robot arm with real-time web controls, inverse kinematics (Modern Robotics), and a combination of stepper, servo, and brushed DC motors. The firmware serves a web UI via SPIFFS and uses WebSockets for low-latency commands, and does kinematics and motor control for the arm.

## Overview
- ESP32 with AsyncWebServer + WebSocket for control
- Inverse kinematics using space/body screw theory via Modern Robotics (Eigen)
- AccelStepper-driven joints with a dedicated run task for smooth motion
- PID control for brushed DC motors with encoders
- Keyboard control and manual joint sliders in the web UI

## Features
- Web UI hosted from SPIFFS (`data/`) with live control
- Keyboard position nudge (W/A/S/D, Q/E) at 10 Hz
- Manual per-joint sliders with labeled readouts (rad + deg)
- Latest-command queue decouples WebSocket ISR from IK/control loop

## Hardware
- Board: ESP32 DOIT DevKit v1
- Motors: 2 Nema 17 stepper motors, 1 kp48fp8g-504 stepper motor, 1 25D Metal Gearmotor with encoder, and 2 MG995R servos.
- Motor Drivers: 2 A4988 stepper drivers, 1 TB6600 stepper driver, 1 H-bridge DC motor driver
- Pin mappings: see `src/robot/motors.h`

## Firmware Architecture
- `src/main.cpp`: Initializes motors, kinematics, SPIFFS, starts web services; runs a `controlTask` on core 0 consuming latest position command from a FreeRTOS queue.
- `src/server/myServer.*`: WiFi, SPIFFS file serving, WebSocket endpoint `/ws`.
- `src/robot/motors.*`: AccelStepper setup; dedicated `stepperRunTask` pinned to core 1; joint setters.
- `src/robot/kinematics.*`: Screw axes, `FKinSpace/Body`, `JacobianSpace/Body`, and `IKinBodyLinear` iterative IK (linear-only target) with trust-region clamp.
- `data/`: Web UI — `index.html`, `style.css`, `script.js`.

## Project Structure
```
robotArm_v2/
	platformio.ini
	data/               # SPIFFS web assets
	src/
		main.cpp
		robot/
			motors.*
			kinematics.*
			modern_robotics.*
			trajectory.*
		server/
			myServer.*
	python/             # notebooks and utilities
```

## Build & Flash
Use PlatformIO tasks or run the commands below.

1) Upload firmware:
```zsh
cd /Users/cole/code/robotArm_v2
pio run --target upload --environment esp32doit-devkit-v1
```

2) Upload SPIFFS web assets:
```zsh
pio run --target uploadfs --environment esp32doit-devkit-v1
```

3) Monitor serial:
```zsh
pio device monitor -b 115200
```

## Web UI
- Served at `http://<esp32-ip>/` after WiFi connects
- Keyboard control: W/A/S/D for X/Y, Q/E for Z
- Joint controls: 4 sliders with labels and readouts; changes send JSON via WebSocket

## Controls Protocol
- WebSocket endpoint: `ws://<esp32-ip>/ws`
- Position JSON (type 0):
```json
{"x": <mm>, "y": <mm>, "z": <mm>, "type": 0}
```
- Joint JSON (type 1):
```json
{"t1": <rad>, "t2": <rad>, "t3": <rad>, "t4": <rad>, "t5": <rad>, "t6": <rad>, "type": 1}
```
- Zero joints (type 2):
```json
{"type": 2}
```

## IK Notes
- `IKinBodyLinear(Tsd, thetalist, ev, maxiter, step_scale)` minimizes position error only (ignores orientation).
- Uses space-frame consistency (`FKinSpace` + `JacobianSpace`) and clamps per-iteration joint step to avoid divergence.
- If error rebounds, consider reducing `step_scale`, increasing `maxiter`, or enabling adaptive scaling.

## Development Tips
- After UI changes under `data/`, re-run `uploadfs`.
- Keep `stepperRunTask` on core 1; avoid heavy work in WebSocket callbacks.
- Use the queue in `main.cpp` to feed the control task with the latest target at ~50 Hz.

## License
Proprietary project components alongside Modern Robotics references; please consult the repository owner for reuse permissions.

