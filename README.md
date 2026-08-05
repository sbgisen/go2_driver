# go2_driver

ROS 2 driver for the Unitree Go2. It is a pair of bridges between the robot's
firmware DDS interface and a standard ROS 2 / Nav2 stack:

| Component | Executable | Direction | Responsibility |
|---|---|---|---|
| `go2_driver::Go2Driver` | `go2_driver_node` | robot &rarr; ROS | Joint states, point cloud, dynamic TF, planar odometry |
| `go2_driver::Go2SportBridge` | `go2_sport_bridge_node` | ROS &rarr; robot | `cmd_vel` and mode services translated to Sport API requests |

Both are `rclcpp_components` plugins in a single shared library, so they can be
composed into a container, and both also ship as standalone executables.

They are deliberately kept separate: the state bridge must keep publishing TF
and odometry even when no one is allowed to command the robot.

## Prerequisites

The Go2 firmware speaks CycloneDDS on `ROS_DOMAIN_ID=0` over the robot's
internal wired network. Nodes join that DDS domain directly — there is no
translation bridge — so `ROS_DOMAIN_ID` must be `0` and `CYCLONEDDS_URI` should
pin the network interface to avoid leaking topics onto other LANs.

Message packages: [`unitree_go`](https://github.com/Unitree-Go2-Robot/unitree_go),
[`unitree_api`](https://github.com/Unitree-Go2-Robot/unitree_api), and
[`go2_interfaces`](https://github.com/sbgisen/go2_interfaces) for the service
definitions.

## Launch

```bash
ros2 launch go2_driver go2_driver.launch.py
```

Every `go2_driver` parameter is exposed as a launch argument. One extra argument
controls what is started:

| Argument | Default | Description |
|---|---|---|
| `enable_sport_bridge` | `true` | Start `go2_sport_bridge_node` as well |

Run the state bridge alone with `enable_sport_bridge:=false`.

---

## `go2_driver` — state bridge

### Interfaces

| Kind | Name | Type | Notes |
|---|---|---|---|
| Subscriber | `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | Renamed by `input_pointcloud_topic` |
| Subscriber | `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | Renamed by `input_odom_topic`, best-effort QoS |
| Subscriber | `lowstate` | `unitree_go/msg/LowState` | |
| Publisher | `pointcloud` | `sensor_msgs/msg/PointCloud2` | Input cloud with `frame_id` and stamp normalised |
| Publisher | `joint_states` | `sensor_msgs/msg/JointState` | Motor states remapped to URDF joint order |
| Publisher | `odom_planar` | `nav_msgs/msg/Odometry` | Renamed by `output_planar_odom_topic` |
| TF | `odom` &rarr; `base_footprint` | | x / y / yaw only |
| TF | `base_footprint` &rarr; `base_link` | | z / roll / pitch only |

The TF chain is split so that `base_footprint` stays a ground-projected frame
while `base_link` follows the body height, which changes as the robot lies
down, stands and walks.

`odom_planar` is the flattened odometry Nav2 consumes: the same pose with roll,
pitch and z removed.

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `input_pointcloud_topic` | string | `/utlidar/cloud` | Unitree L1 point cloud topic |
| `input_odom_topic` | string | `/utlidar/robot_odom` | Unitree odometry topic |
| `output_planar_odom_topic` | string | `odom_planar` | Flattened odometry topic |
| `pointcloud_frame` | string | `utlidar_lidar` | `frame_id` written onto the republished cloud |
| `odom_frame` | string | `odom` | Parent of the published TF chain |
| `base_footprint_frame` | string | `base_footprint` | Ground-projected frame |
| `base_link_frame` | string | `base_link` | Robot body frame |
| `body_z_offset` | double | `0.0` | Offset added to the `base_link` body height [m] |
| `use_msg_stamp` | bool | `false` | Stamp output with the input stamp instead of the node clock |
| `publish_tf` | bool | `true` | Broadcast the TF chain |
| `publish_planar_odom` | bool | `true` | Publish the flattened odometry |

`pointcloud_frame` must name a frame that actually exists in the URDF, since
this node only relabels the cloud — it does not publish that frame itself.

`publish_tf` and `publish_planar_odom` exist so the driver can coexist with
another producer of the same frames or topic. Leaving both enabled while a
second node publishes the same child frame gives two publishers on `/tf`, which
makes the transform jitter between the two sources.

---

## `go2_sport_bridge` — command bridge

Translates ROS commands into `unitree_api/msg/Request` messages on
`api/sport/request`. Publishing is fire-and-forget: the bridge does not wait for
`api/sport/response`, so `success` means "the request was published", not "the
robot accepted it".

### Interfaces

| Kind | Name | Type | Description |
|---|---|---|---|
| Publisher | `api/sport/request` | `unitree_api/msg/Request` | Sport API requests |
| Subscriber | `cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command |
| Service | `mode` | `go2_interfaces/srv/Mode` | Run a named preset |
| Service | `speed_level` | `go2_interfaces/srv/SpeedLevel` | Set the movement speed level |
| Service | `switch_joystick` | `go2_interfaces/srv/SwitchJoystick` | Enable / disable the stock remote |

The bridge declares no parameters.

### `cmd_vel`

`geometry_msgs/msg/Twist` maps straight onto the Sport API `Move` command.

| Twist field | `Move` parameter |
|---|---|
| `linear.x` | `x` |
| `linear.y` | `y` |
| `angular.z` | `z` |

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

The robot has to be in a walkable mode (for example `stop_move`) before it
reacts to a velocity command.

### `mode`

Takes one preset name and runs the matching Sport API command, or a short
sequence of them. An unknown name returns `success: false` with the full list of
valid presets in `message`.

```bash
# stand up from damp
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'recovery_stand'}"

# lie down and relax safely (recommended shutdown sequence)
ros2 service call /mode go2_interfaces/srv/Mode "{mode: 'stand_down_damp'}"
```

Basic postures and gestures:

| Preset | Description |
|---|---|
| `damp` | Relax all joints |
| `balance_stand` | Balance standing |
| `stop_move` | Return to normal walking mode and stop |
| `stand_down` | Lie down |
| `recovery_stand` | Stand up from damp |
| `sit` / `rise_sit` | Sit / stand up from sitting |
| `hello` | Wave |
| `stretch` | Stretch |
| `dance1` / `dance2` | Dance |
| `finger_heart` | Finger heart |

Gaits:

| Preset | Description |
|---|---|
| `static_walk` | Static walk |
| `trot_run` | Trot run |
| `economic_gait` | Low-power gait |
| `classic_walk` | Classic walk — the gait that climbs stairs |
| `free_walk` | Free walk — the gait that descends stairs, facing forwards |
| `free_bound` | Free bound gait |
| `free_jump` | Free jump gait |
| `free_avoid` | Obstacle-avoiding gait |
| `switch_avoid_mode` | Toggle the obstacle avoidance mode |
| `auto_recovery_set` | Enable automatic recovery after a fall |

Acrobatics — only on a clear, safe surface:

| Preset | Description |
|---|---|
| `walk_upright` | Walk on the hind legs |
| `cross_step` | Walk on two crossed legs |
| `left_flip` | Flip to the left |
| `back_flip` | Backflip |
| `hand_stand` | Handstand |

Sequences:

| Preset | Description |
|---|---|
| `stand_down_damp` | `stand_down` &rarr; wait 2 s &rarr; `damp` |

`stand_up` (Sport API 1004) is intentionally not exposed; use `recovery_stand`
or `balance_stand`.

Sport API IDs removed in [unitree_ros2 v0.2.0](https://github.com/unitreerobotics/unitree_ros2/releases/tag/v0.2.0)
— `SwitchGait`, `Trigger`, `BodyHeight`, `FootRaiseHeight`, `TrajectoryFollow`,
`ContinuousGait`, `Wallow` and the matching getters — are not implemented and
must not be added back: current firmware ignores them. They are listed in
`include/go2_driver/sport_api_id.hpp` so their values are not reused.

### `speed_level`

```bash
ros2 service call /speed_level go2_interfaces/srv/SpeedLevel "{level: 1}"
```

`level` must be `-1` or `1`; anything else is rejected with `success: false`.
`0` is **not** valid even though it sits between them — the firmware answers it
with `status.code: -1` on `/api/sport/response`. Note that `success: true` only
means the Sport API request was published, not that the robot accepted it.

### `switch_joystick`

```bash
ros2 service call /switch_joystick go2_interfaces/srv/SwitchJoystick "{flag: true}"
```

## Debugging

| Topic | Contents |
|---|---|
| `/lf/sportmodestate` | Sport mode state reported by the robot |
| `/api/sport/request` | Requests this package sent |
| `/api/sport/response` | Replies from the robot; `status.code == 0` means success |

## License

Apache-2.0. This package derives from
[Unitree-Go2-Robot/go2_driver](https://github.com/Unitree-Go2-Robot/go2_driver)
by Intelligent Robotics Lab (URJC); see `NOTICE` for the original BSD 3-Clause
terms, which continue to apply to the files that came from it.
