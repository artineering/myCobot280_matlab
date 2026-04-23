# MyCobot 280 M5 - MATLAB Interface

MATLAB class for controlling the [Elephant Robotics myCobot 280 M5](https://docs.elephantrobotics.com/docs/mycobot_280_m5_en/) robotic arm via serial UART.

## Hardware

- **Robot**: myCobot 280 M5 (6-DOF, 280mm reach, 250g payload)
- **Controller**: M5Stack Basic (base) + M5Stack Atom (end-effector)
- **Connection**: USB-C serial (CH9102 chip), 115200 baud
- **Firmware**: M5Stack Basic must be running **Transponder** mode

## Joint Limits

Measured safe operating ranges for this unit:

| Joint | Min | Max |
|-------|-----|-----|
| J1 | -140 | +150 |
| J2 | -90 | +90 |
| J3 | -145 | +150 |
| J4 | -90 | +90 |
| J5 | -145 | +150 |
| J6 | -175 | +175 |

Angles outside these ranges are rejected by the class before sending to the robot.

## Quick Start

```matlab
% Connect
myc = MyCobot280('COM13', 'BaudRate', 115200);
myc.powerOn();
pause(1);

% Read current state
myc.getAngles()    % [j1 j2 j3 j4 j5 j6] in degrees
myc.getCoords()    % [x y z rx ry rz] in mm/degrees

% Move joints
myc.sendAngles([0 0 0 0 0 0], 20);   % home position, speed 20
pause(3);
myc.sendAngle(1, 45, 25);            % J1 to 45 degrees

% Cartesian move
myc.sendCoords([160 0 200 0 0 0], 20, 1);  % linear mode

% Gripper
myc.setGripperState(0, 30);   % open
myc.setGripperState(1, 30);   % close

% LED
myc.setColor(0, 255, 0);      % green

% Free move (release servos)
myc.setFreeMove(true);

% Disconnect
myc.powerOff();
delete(myc);
```

## Constructor Options

```matlab
myc = MyCobot280(port, Name, Value)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BaudRate` | 115200 | Serial baud rate |
| `Timeout` | 5 | Read timeout in seconds |
| `Debug` | false | Print TX/RX bytes to console |
| `MaxRetries` | 3 | Retry count on communication failure |
| `CommandDelay` | 0.05 | Minimum delay between commands (seconds) |

## API Reference

### Power & Status
| Method | Description |
|--------|-------------|
| `powerOn()` | Power on the robot |
| `powerOff()` | Power off the robot |
| `isPoweredOn()` | Check if powered on |
| `isControllerConnected()` | Check controller connection |
| `isReady()` | Check if robot responds |
| `isMoving()` | Check if currently moving |

### Joint Control
| Method | Description |
|--------|-------------|
| `getAngles()` | Get all joint angles (1x6 degrees) |
| `sendAngle(joint, angle, speed)` | Move single joint (1-6) |
| `sendAngles(angles, speed)` | Move all joints (1x6 vector) |
| `waitForIdle(timeout)` | Block until motion completes |

### Cartesian Control
| Method | Description |
|--------|-------------|
| `getCoords()` | Get end-effector pose [x y z rx ry rz] |
| `sendCoord(axis, value, speed)` | Move single axis (1-6) |
| `sendCoords(coords, speed, mode)` | Move to pose (mode: 0=angular, 1=linear) |

### Jog Control
| Method | Description |
|--------|-------------|
| `jogAngle(joint, direction, speed)` | Jog joint (direction: 0/1) |
| `jogCoord(axis, direction, speed)` | Jog axis (direction: 0/1) |
| `jogStop()` | Stop jogging |

### Gripper
| Method | Description |
|--------|-------------|
| `getGripperValue()` | Read gripper opening (0-100) |
| `setGripperState(state, speed)` | Open (0) or close (1) |
| `setGripperValue(value, speed)` | Set specific opening (0-100) |
| `isGripperMoving()` | Check if gripper is moving |

### Servo Control
| Method | Description |
|--------|-------------|
| `isServoEnabled(joint)` | Check if servo is enabled |
| `isAllServoEnabled()` | Check all servos |
| `focusServo(joint)` | Power on a servo |
| `releaseServo(joint)` | Power off a servo |
| `setFreeMove(enable)` | Enable/disable free move mode |

### LED & IO
| Method | Description |
|--------|-------------|
| `setColor(r, g, b)` | Set Atom LED color (0-255 each) |
| `setPinMode(pin, mode)` | Set pin mode (0=input, 1=output) |
| `setDigitalOutput(pin, level)` | Set digital output |
| `getDigitalInput(pin)` | Read digital input |

### Coordinate Frames
| Method | Description |
|--------|-------------|
| `setToolReference(coords)` | Set tool coordinate system |
| `getToolReference()` | Get tool coordinate system |
| `setReferenceFrame(type)` | Set frame (0=base, 1=world) |
| `setEndType(type)` | Set end type (0=flange, 1=tool) |

## Protocol

Communication uses a binary serial protocol:

```
[0xFE] [0xFE] [LEN] [CMD] [DATA...] [0xFA]
  header        |     |       |       footer
              length  cmd   payload
```

- Angles are encoded as signed 16-bit big-endian, multiplied by 100
- XYZ coordinates are multiplied by 10, rotations by 100
- The class implements retry logic (up to 3 attempts) and frame re-synchronization to recover from communication errors

## Files

| File | Description |
|------|-------------|
| `MyCobot280.m` | Main MATLAB class |
| `test_joint_limits.m` | Joint limit validation test |
| `pymycobot/` | Elephant Robotics Python SDK (reference) |

## Requirements

- MATLAB R2019b or later (uses `serialport`)
- myCobot 280 M5 with Transponder firmware on M5Stack Basic
- USB-C cable to robot base
