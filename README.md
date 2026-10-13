# go2_driver

ROS 2 driver for the Unitree Go2. It bridges the robot's firmware DDS
interface to a standard ROS 2 / Nav2 stack:

| Component | Executable | Direction | Responsibility |
|---|---|---|---|
| `go2_driver::Go2Driver` | `go2_driver_node` | robot &rarr; ROS | Joint states, point cloud, dynamic TF, odometry |
| `go2_driver::Go2SportBridge` | `go2_sport_bridge_node` | ROS &rarr; robot | `cmd_vel` and mode services translated to Sport API requests |
| `go2_driver::Go2Camera` | `go2_camera_node` | robot &rarr; ROS | Front camera H.264 stream decoded to `sensor_msgs/Image` |

All three are `rclcpp_components` plugins in a single shared library, so they
can be composed into a container, and each also ships as a standalone
executable.

They are deliberately kept separate: the state bridge must keep publishing TF
and odometry even when no one is allowed to command the robot, and a camera
decode stall must not take the state bridge down with it.

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

Every `go2_driver` parameter is exposed as a launch argument. The launch file
starts both bridges and the camera node; `use_camera:=false` leaves the camera
out.

Nothing else may own `api/sport/request` at the same time: two command bridges
would both subscribe to `cmd_vel` and both push Sport API requests, and the
robot would act on the merged stream. Stop the other bridge before adopting
this launch file, or write your own launch file that starts only
`go2_driver_node`.

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
| Publisher | `odometry/lio` | `nav_msgs/msg/Odometry` | Unitree odometry republished with `child_frame_id` = `base_link` |
| TF | `odom` &rarr; `base_footprint` | | x / y / yaw only |
| TF | `base_footprint` &rarr; `base_link` | | z / roll / pitch only |

The TF chain is split so that `base_footprint` stays a ground-projected frame
while `base_link` follows the body height, which changes as the robot lies
down, stands and walks.

`odometry/lio` is the Unitree odometry republished as-is in the `odom` frame,
with `body_z_offset` applied to z and the covariance the firmware leaves at zero
filled in.

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `input_pointcloud_topic` | string | `/utlidar/cloud` | Unitree L1 point cloud topic |
| `input_odom_topic` | string | `/utlidar/robot_odom` | Unitree odometry topic |
| `pointcloud_frame` | string | `utlidar_lidar` | `frame_id` written onto the republished cloud |
| `odom_frame` | string | `odom` | Parent of the published TF chain |
| `base_footprint_frame` | string | `base_footprint` | Ground-projected frame |
| `base_link_frame` | string | `base_link` | Robot body frame |
| `body_z_offset` | double | `0.0` | Offset added to the `base_link` body height [m] |
| `use_msg_stamp` | bool | `false` | Stamp output with the input stamp instead of the node clock |
| `publish_tf` | bool | `true` | Broadcast the TF chain |
| `publish_odom` | bool | `true` | Republish the Unitree odometry |

`pointcloud_frame` must name a frame that actually exists in the URDF, since
this node only relabels the cloud — it does not publish that frame itself.

`publish_tf` and `publish_odom` exist so the driver can coexist with
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

## `go2_camera` — front camera

Decodes the H.264 stream the firmware publishes on `/frontvideostream`
(`unitree_go/msg/Go2FrontVideoData`) and republishes it as plain images.
Decoding goes through `ffmpeg_encoder_decoder` (libavcodec), so no FFmpeg
development packages have to be installed by hand: `rosdep install` pulls in
everything.

### Interfaces

| Kind | Name | Type | Notes |
|---|---|---|---|
| Subscriber | `/frontvideostream` | `unitree_go/msg/Go2FrontVideoData` | Renamed by `input_video_topic`, best-effort QoS |
| Publisher | `go2_camera/image_raw` | `sensor_msgs/msg/Image` | `bgr8`; `image_transport` plugins add `compressed` etc. |
| Publisher | `go2_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Loaded from `camera_info_url` |
| Service | `set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | Provided by `camera_info_manager` |

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `input_video_topic` | string | `/frontvideostream` | Firmware video topic |
| `resolution` | int | `720` | Only frames whose `resolution` field matches are decoded. The firmware was seen sending `360` and `720`; any other value decodes nothing and the node warns |
| `frame_id` | string | `go2_camera_optical_frame` | `frame_id` of the image and camera info (defined in `go2_description`) |
| `decoder` | string | `h264` | libavcodec decoder name, e.g. `h264_cuvid` for hardware decoding |
| `camera_name` | string | `go2_camera` | Camera name handed to `camera_info_manager` |
| `camera_info_url` | string | `package://go2_driver/config/go2_camera.yaml` | Calibration file URL (`file://` or `package://`) |

The bundled `config/go2_camera.yaml` holds nominal 1280x720 intrinsics taken
from the upstream driver, measured on a different Go2. They are fine for
viewing and rough geometry; for metric work calibrate this camera (e.g. with
`camera_calibration`) and point `camera_info_url` at the result, or let
`set_camera_info` overwrite the bundled file.

Decoding only runs while something subscribes to `go2_camera/image_raw` or one
of its transports, and it always (re)starts at a keyframe, so the first image
can lag a new subscriber by up to one GOP.

Raw 1280x720 `bgr8` frames are 2.7 MB each. To watch the camera from another
machine, pick the `compressed` transport in rviz's Image display instead of
`raw`.

Image stamps are the node clock at decode time; the firmware's `time_frame`
field is not used.

---

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
