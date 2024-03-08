# PuddleDuck
Motor control library for duckling droids. It is true that with great torque comes great responsibility.

<img align="left" width="256" height="256" src="https://github.com/reeltwo/PuddleDuck/assets/16616950/ae5ef098-fab6-44e4-a5b8-ee2fc14e3487">

### Goals ###

- Safety Framework for High Torque
- Joint Description in YAML
- Joint Positions Between 0.0 - 1.0
- Joints can be Inverted
- Safely Update Joint Limits
- Secure Motors from Bad Commands
- Motion Capture
---
### Supported Motors ###

- Go Motors
- Cybeargear (in progress)

### Compiling

```bash
git clone https://github.com/reeltwo/PuddleDuck
cd PuddleDuck
mkdir build && cd build
cmake .. && make
```

### Usage

Default robot configuration can be found in default.yaml it is used to initialize the working copy found in robot.yaml. Joint limits will be saved in robot.yaml.

```bash
./puddle
```

If any motors or adapters are missing it will report an error and quit.

```bash
No configuration file. Loading defaults.
Error opening serial port /dev/ttyUSB1: No such file or directory
Missing left motors
  [4] hip.roll
  [5] hip.yaw
```

If you expect those motors to be missing you can rerun using:

```bash
./puddle -f
```

This time it will report uninitialized ranges for various joints.

```bash
Error opening serial port /dev/ttyUSB1: No such file or directory
Missing left motors
  [4] hip.roll
  [5] hip.yaw
Missing right motors
  [1] ankle.pitch
  [2] knee.pitch
  [3] hip.pitch
  [4] hip.roll
  [5] hip.yaw
FORCE CONTINUE EVEN THOUGH MOTORS ARE MISSING
[0] neck: RANGE UNINITIALIZED
[4] left.hip.roll: RANGE UNINITIALIZED
[5] left.hip.yaw: RANGE UNINITIALIZED
[1] right.ankle.pitch: RANGE UNINITIALIZED
[2] right.knee.pitch: RANGE UNINITIALIZED
[3] right.hip.pitch: RANGE UNINITIALIZED
[4] right.hip.roll: RANGE UNINITIALIZED
[5] right.hip.yaw: RANGE UNINITIALIZED
```

You can manually move all the joints that are connected and type 'c' to save the configuration.

### Keyboard mapping

Here is the keyboard mapping for the 'puddle' example:

- 'a': Stand (stiffen leg joints)
- 'c': Save joint range limits
- 'q': Quit
- 'p': Playback motion recording
- 'r': Record motion
- 's': First time save current stance and stand (stiffen leg joints). Next time assume first stance.
- 'x': Move left ankle to position 0.0
- 'z': Move left ankle to position 1.0

### Robot configuration

```yaml
bus:
  -
    name: left_bus
    type: GoMotor
    adapter: /dev/ttyUSB0
    version: 1
  -
    name: right_bus
    type: GoMotor
    adapter: /dev/ttyUSB1
    version: 1
neck:
  bus: left_bus
  id: 0
  range: [.nan, .nan]
  kp: 1.0
  kd: 0.01
  tau: 0
  invert: false
leg:
  left:
    bus: left_bus
    ankle:
      pitch:
        id: 1
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
    knee:
      pitch:
        id: 2
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
    hip:
      pitch:
        id: 3
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
      roll:
        id: 4
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
      yaw:
        id: 5
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
  right:
    bus: right_bus
    ankle:
      pitch:
        id: 1
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
    knee:
      pitch:
        id: 2
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
    hip:
      pitch:
        id: 3
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
      roll:
        id: 4
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
      yaw:
        id: 5
        range: [.nan, .nan]
        kp: 1.0
        kd: 0.01
        tau: 0
        invert: false
```
