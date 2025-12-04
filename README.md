# ROS2 PPM Bridge

Bridges ROS2 Joy messages to PPM signals for fixed-wing UAV control. The node subscribes to Joy command topics, packs them into PPM channel values, and streams them over serial to a microcontroller that generates the PPM waveform (verified with Arduino Nano ESP32).

## Architecture

| Stage | Component                          | Interface              | Purpose                              |
|------:|------------------------------------|------------------------|--------------------------------------|
| 1     | ROS 2 topics `/joy_throttle`, `/auto_joy_throttle` | `sensor_msgs/msg/Joy`  | Input commands (throttled) |
| 2     | `ppm_bridge_node`                  | ROS 2 ↔︎ serial        | Map Joy → PPM μs values with configurable channel order |
| 3     | Serial link                        | `/dev/ttyACM0` 57600   | Send packed channel bytes            |
| 4     | Arduino Nano ESP32 (PPM encoder)   | PPM output pin         | Generate PPM frame at 22ms interval  |
| 5     | External TX (e.g., Spektrum DSM)   | Trainer/PPM input      | RF transmission to receiver on UAV   |


## Requirements

* ROS 2 Jazzy on Ubuntu 24.04
* `rclcpp`, `std_msgs`, `sensor_msgs`
* Arduino Nano ESP32 or compatible PPM encoder
* Serial connection (`/dev/ttyACM0`) between computer and PPM encoder
* USB joystick (optional, for manual control)

## Building

```bash
cd ~/ros2_ws/src
git clone https://github.com/cognipilot/ppm_bridge.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ppm_bridge
source install/setup.bash
```

## Usage

Launch files are provided for different vehicles:

**Sport Cub S 2** (4-channel, AETRM order):
```bash
ros2 launch ppm_bridge cub_vehicle_bridge.launch.xml vehicle:=cub1
```

**UMX Night Vapor** (3-channel, TAERM order):
```bash
ros2 launch ppm_bridge nvp_vehicle_bridge.launch.xml vehicle:=nv1
```

**Basic demo** (TAERM order):
```bash
ros2 launch ppm_bridge bridge.launch.xml
```

## Configuration

The node uses a `channel_map` parameter to reorder outputs for different vehicle requirements:

**Internal servo_data indices:**
- `0` = Throttle
- `1` = Aileron  
- `2` = Elevator
- `3` = Rudder
- `4` = Mode

**Channel mapping:** `output[i] = servo_data[channel_map[i]]`

**Common configurations:**
- **TAERM** (Throttle, Aileron, Elevator, Rudder, Mode): `[0, 1, 2, 3, 4]` - Night Vapor, default
- **AETRM** (Aileron, Elevator, Throttle, Rudder, Mode): `[1, 2, 0, 3, 4]` - Sport Cub S 2

## Topics

**Subscribed:**
- `joy_throttle` (`sensor_msgs/msg/Joy`) - Manual control input (throttled to 20 Hz)
- `auto_joy_throttle` (`sensor_msgs/msg/Joy`) - Autonomous control input (throttled to 20 Hz)

**Published:**
- `status` (`std_msgs/msg/String`) - Human-readable channel values
- `joy_serial_status` (`std_msgs/msg/UInt16MultiArray`) - Channel values in configured order (1000-2000 μs)

## Parameters

- `controller_id` (string) - Controller type: `"taranis"` or `"f310"` (default: unset)
- `channel_map` (int array) - Output channel order (default: `[0, 1, 2, 3, 4]`)

## Joy Message → PPM Mapping

Internal servo_data is always stored as [Throttle, Aileron, Elevator, Rudder, Mode]:

| Index | Channel   | Joy value → PPM (μs)                    | Notes                                |
|------:|-----------|-----------------------------------------|--------------------------------------|
| 0     | Throttle  | `0.0 → 1000`, `1.0 → 2000`              | `0.0` = idle, `1.0` = full throttle  |
| 1     | Aileron   | `-1.0 → 1000`, `0.0 → 1500`, `1.0 → 2000` | `+` = left, `-` = right            |
| 2     | Elevator  | `-1.0 → 2000`, `0.0 → 1500`, `1.0 → 1000` | `+` = pitch down, `-` = pitch up   |
| 3     | Rudder    | `-1.0 → 1000`, `0.0 → 1500`, `1.0 → 2000` | `+` = yaw left, `-` = yaw right    |
| 4     | Mode      | `-1.0 → 1000`, `1.0 → 2000`             | `-1.0` = manual, `1.0` = stabilized  |

The `channel_map` parameter determines how these are reordered for serial output.
