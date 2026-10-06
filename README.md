# ROS 2 PID-Based Right-Wall Following

A Python mobile robot controller that uses laser scan measurements
to follow a wall on the robot's right-hand side.

The controller targets a wall distance of 0.25 metres and overrides
wall following when an obstacle is detected ahead.

## How It Works

1. Read laser measurements from `/scan`.
2. Find the nearest positive distance in the right-side scan sector.
3. Calculate the error:

   error = target distance − measured distance

4. Use proportional, integral, and derivative terms to calculate steering.
5. Publish movement commands to `/cmd_vel`.

When either front scan sector detects a distance below 0.5 metres,
the robot stops moving forward and receives an angular velocity
command of 0.5 rad/s.

## Controller Settings

| Parameter | Value |
|-----------|-------|
| Target wall distance | 0.25 m |
| Proportional gain, Kp | 1.0 |
| Integral gain, Ki | 0.0 |
| Derivative gain, Kd | 0.2 |
| Normal forward speed | 0.1 m/s |
| Front-obstacle threshold | 0.5 m |
| Obstacle turn command | 0.5 rad/s |
| Command publishing rate | 5 Hz |

The integral gain is zero, so the current configuration operates
as a PD controller.

## Main File

`PID.py` — laser processing, controller calculations,
obstacle response, and velocity publishing.

## ROS Interfaces

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Laser measurements |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Movement commands |

Node name: `right_edge_following`

## Requirements

- Python 3.
- A configured ROS 2 environment.
- Packages providing `rclpy`, `sensor_msgs`, `geometry_msgs`,
  `nav_msgs`, and `tf2_ros`.
- A robot or simulation publishing `/scan` and accepting `/cmd_vel`.

## Running

Start the robot or simulation first.

In a terminal with your ROS 2 environment sourced, navigate
to the directory containing the script and run:

```bash
python3 PID.py
```

Press `Ctrl+C` to stop the controller.

## Configuration

Adjust `Kp`, `Ki`, `Kd`, and `target_distance` near the beginning
of the script to tune the controller.

Verify that the fixed laser scan indices match your scanner:

- Right: `265:275`
- Front: `0:5` and `355:360`
