# go2_driver

ROS 2 driver for the Unitree Go2. It bridges the robot's firmware DDS interface
and a standard ROS 2 / Nav2 stack:

| Component | Executable | Direction | Responsibility |
|---|---|---|---|
| `go2_driver::Go2Driver` | `go2_driver_node` | robot &rarr; ROS | Joint states, point cloud, dynamic TF, odometry |
| `go2_driver::Go2SportBridge` | `go2_sport_bridge_node` | ROS &rarr; robot | `cmd_vel` and motion services translated to Sport API requests |
| `go2_driver::Go2RobotStateBridge` | `go2_robot_state_bridge_node` | ROS &rarr; robot | Start, stop and list the services running inside the robot |
| `go2_driver::Go2Camera` | `go2_camera_node` | robot &rarr; ROS | Front camera H.264 stream decoded to `sensor_msgs/Image` |

All four are `rclcpp_components` plugins in a single shared library, so they
can be composed into a container, and all four also ship as standalone
executables.

They are deliberately kept apart. The state bridge must keep publishing TF and
odometry even when no one is allowed to command the robot, and the robot state
API IDs collide numerically with the sport ones — `SERVICE_SWITCH` and `DAMP`
are both 1001 — so the two command bridges never share an enum or a publisher.
A camera decode stall must not take the state bridge down with it either.

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
starts all four nodes; `use_camera:=false` leaves the camera out.

Nothing else may own `api/sport/request` at the same time: two command bridges
would both subscribe to `cmd_vel` and both push Sport API requests, and the
robot would act on the merged stream. Stop the other bridge before adopting
this launch file, or write your own launch file that starts only the nodes you
want.

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
`api/sport/request`.

Every service waits for the robot's own reply on `api/sport/response`, so
`success` means the robot reported status code 0 — not merely that a message
went out. A request the robot never answers fails once the timeout expires.

### Interfaces

| Kind | Name | Type | Description |
|---|---|---|---|
| Publisher | `api/sport/request` | `unitree_api/msg/Request` | Sport API requests |
| Subscriber | `api/sport/response` | `unitree_api/msg/Response` | Replies, matched to requests by `header.identity.id` |
| Subscriber | `cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command |
| Service | `mode` | `go2_interfaces/srv/Mode` | Run a named preset |
| Service | `speed_level` | `go2_interfaces/srv/SpeedLevel` | Set the movement speed level |
| Service | `switch_joystick` | `go2_interfaces/srv/SwitchJoystick` | Enable / disable the stock remote |
| Service | `euler` | `go2_interfaces/srv/Euler` | Body attitude while standing and walking (see `pose`) |
| Service | `pose` | `go2_interfaces/srv/Pose` | Enter / leave pose mode |
| Service | `get_auto_recovery` | `go2_interfaces/srv/GetAutoRecovery` | Whether the robot stands up by itself after a fall |

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `wait_for_response` | bool | `true` | Report the robot's status code instead of only that the request was published |
| `response_timeout` | double | `2.0` | Seconds the robot is given to reply [s] |
| `blocked_modes` | string[] | `[front_flip, back_flip, left_flip, hand_stand]` | `mode` presets refused without being sent; `[none]` blocks nothing (see [`mode`](#mode)) |

Set `wait_for_response:=false` if a firmware version turns out not to answer a
request you need; services then return `success: true` as soon as the request is
published, and say "published" rather than "accepted" in `message`.
`get_auto_recovery` cannot work in that mode, because its answer *is* the reply.

All three parameters are launch arguments: `wait_for_response`, `blocked_modes`
and `sport_response_timeout` (this node's `response_timeout`).
`go2_robot_state_bridge` has no `wait_for_response` — it always waits, since
each of its services is a query — and its `response_timeout` is the separate
`robot_state_response_timeout` argument, because the two bridges want
different values. `blocked_modes` takes a yaml list:

```bash
ros2 launch go2_driver go2_driver.launch.py blocked_modes:="[front_flip, back_flip]"
```

### `cmd_vel`

`geometry_msgs/msg/Twist` maps straight onto the Sport API `Move` command.

| Twist field | `Move` parameter |
|---|---|
| `linear.x` | `x` |
| `linear.y` | `y` |
| `angular.z` | `z` (yaw rate) |

The velocities are in the body frame, and the firmware clamps them to
`x` [-2.5, 3.8] m/s, `y` [-1.0, 1.0] m/s, `z` [-4, 4] rad/s.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

The robot has to be in a walkable mode (for example `stop_move`) before it
reacts to a velocity command.

**The robot holds the last `Move` for one second and does not filter it.**
Unitree documents both properties, so a `cmd_vel` stream that simply stops
leaves the robot walking for up to a second. Send `Move(0, 0, 0)` — a zero
`Twist` — or call `mode: 'stop_move'` when you stop steering, and filter the
command before publishing rather than relying on the firmware to smooth it.

### `mode`

Takes one preset name and runs the matching Sport API command, or a short
sequence of them. An unknown name returns `success: false` with the full list of
valid presets in `message`.

A sequence runs one step per robot reply and does not block the node, so the
other services keep answering while it is in flight. Only one sequence runs at a
time: a second `mode` call during one is rejected rather than queued.

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
| `content` | Look pleased |
| `scrape` | Scrape a front paw |
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

Acrobatics — only on a clear, safe surface. The presets listed in the
`blocked_modes` parameter are **refused**: `mode` answers `success: false`
without sending anything. By default those are `front_flip`, `back_flip`,
`left_flip` and `hand_stand`, which put excessive stress on the robot and its
payload. Shorten or extend the list per robot, or publish the Sport API id on
`api/sport/request` yourself if you really mean it. To block nothing, pass
`blocked_modes:="[none]"` — `none` is the documented stand-in for an empty
list, which ROS 2 cannot express as a parameter value (`[]` makes the node
exit while reading its parameters). Any other name that is no preset is warned
about and ignored.

| Preset | Description |
|---|---|
| `walk_upright` | Walk on the hind legs |
| `cross_step` | Walk on two crossed legs |
| `front_flip` | Front flip |
| `front_jump` | Jump forwards |
| `front_pounce` | Pounce forwards |
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

### `euler` and `pose`

`euler` sets the body attitude, in radians, with the firmware clamping to
`roll` and `pitch` [-0.75, 0.75] and `yaw` [-0.6, 0.6].

**Call `pose` with `flag: true` first.** Outside pose mode the robot accepts the
request and reports success, but the attitude either does not change or springs
back: the movement peaks after about half a second and is level again after one,
so a single `ros2 topic echo` of the resulting TF will usually miss it entirely.

```bash
ros2 service call /pose go2_interfaces/srv/Pose "{flag: true}"
ros2 service call /euler go2_interfaces/srv/Euler "{roll: 0.0, pitch: 0.3, yaw: 0.0}"
ros2 service call /pose go2_interfaces/srv/Pose "{flag: false}"
```

### `speed_level`

```bash
ros2 service call /speed_level go2_interfaces/srv/SpeedLevel "{level: 1}"
```

`level` must be `-1` or `1`; anything else is rejected with `success: false`.
`0` is **not** valid even though it sits between them — Unitree documents it as
"normal speed", but the firmware answers it with `status.code: -1` (measured 6/6
on 2026-08-05 and 3/3 again on 2026-08-26).

### `switch_joystick`

```bash
ros2 service call /switch_joystick go2_interfaces/srv/SwitchJoystick "{flag: true}"
```

`go2_interfaces/srv/SwitchJoystick` and `srv/Pose` carry no `message` field, so
when one of them fails the reason is logged instead of returned.

---

## `go2_robot_state_bridge` — robot service control

Talks to the Robot State API on `api/robot_state/request`, which starts, stops
and lists the services running inside the robot. Added in unitree_ros2 v0.2.0.

Starting this node is harmless on its own — it only exposes the services. The
risk is in calling them: `service_switch` can stop the robot's own services,
`sport_mode` included, which disables the walking controller.

### Interfaces

| Kind | Name | Type | API ID | Description |
|---|---|---|---|---|
| Publisher | `api/robot_state/request` | `unitree_api/msg/Request` | | Robot State API requests |
| Subscriber | `api/robot_state/response` | `unitree_api/msg/Response` | | Replies, matched by `header.identity.id` |
| Service | `service_switch` | `go2_interfaces/srv/ServiceSwitch` | 1001 | Start or stop one of the robot's services |
| Service | `set_report_freq` | `go2_interfaces/srv/SetReportFreq` | 1002 | How often the robot reports its service state |
| Service | `service_list` | `go2_interfaces/srv/ServiceList` | 1003 | List the services with their status and protect flags |

Parameter `response_timeout` (double, default `5.0`) — more generous than the
sport bridge's, since listing takes longer than acknowledging. The launch file
exposes it as `robot_state_response_timeout`.

```bash
# what is running
ros2 service call /service_list go2_interfaces/srv/ServiceList "{}"

# restart a service that was switched off
ros2 service call /service_switch go2_interfaces/srv/ServiceSwitch "{name: 'ota_box', enable: true}"

# report the service state every 3 s for the next 30 s
ros2 service call /set_report_freq go2_interfaces/srv/SetReportFreq "{interval: 3, duration: 30}"
```

**`status` is 0 for running and 1 for stopped** — the opposite of what the
numbers suggest. `enable: true` therefore drives `status` from 1 to 0.
`service_list` reports -1 for an entry whose status the robot left out.

`service_list` reports `name`, `status` and `protect` only. It carries no
version field; the robot's own `/servicestate` topic does, and
`set_report_freq` is what makes that topic flow.

Services whose `protect` flag is set cannot be switched off; the robot answers
5202. A switch that fails for any other reason answers 5201.

Which service drives the legs depends on the firmware: `sport_mode` below
V1.1.6, `mcf` from V1.1.6 on. Both names appear in `service_list`, so read the
`status` rather than the presence of a name to tell which one is live. Switching
the live one off is not a way to make the robot safe — it keeps accepting sport
commands, and the flag returns on its own.

The robot sends no reply to `set_report_freq`, so that one alone answers as soon
as the request is published. Its effect is visible on the robot's own
`/servicestate` topic, which carries the same list as a JSON string.

---

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
| `resolution` | int | `720` | Only frames whose `resolution` field matches are decoded; the stream carries several |
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

## Debugging

| Topic | Contents |
|---|---|
| `/lf/sportmodestate` | Sport mode state reported by the robot |
| `/servicestate` | The robot's own service-state report, as a JSON string |
| `/api/sport/request` | Requests this package sent |
| `/api/sport/response` | Replies from the robot; `status.code == 0` means success |

If every service call comes back with "got no reply", check that the robot
really publishes the response topic and that the QoS is compatible:

```bash
ros2 topic info /api/sport/response --verbose
ros2 topic info /api/robot_state/response --verbose
```

## License

Apache-2.0. This package derives from
[Unitree-Go2-Robot/go2_driver](https://github.com/Unitree-Go2-Robot/go2_driver)
by Intelligent Robotics Lab (URJC); see `NOTICE` for the original BSD 3-Clause
terms, which continue to apply to the files that came from it.
