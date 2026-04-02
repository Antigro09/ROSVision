# ROSVision

Low-latency FRC vision system for the **Jetson Orin Nano Super** (or any
CUDA-capable Jetson) running **ROS 2 Humble/Iron**.

Uses **NVIDIA Isaac ROS AprilTag** for CUDA-accelerated tag detection and
publishes pose results to **WPILib NetworkTables 4** in PhotonVision-compatible
conventions.

---

## Table of Contents

1. [Architecture overview](#architecture-overview)
2. [Coordinate frame conventions](#coordinate-frame-conventions)
3. [Hardware & software requirements](#hardware--software-requirements)
4. [Installing dependencies](#installing-dependencies)
5. [Build](#build)
6. [Configuration](#configuration)
7. [Running](#running)
8. [NetworkTables schema](#networktables-schema)
9. [Field layout JSON format](#field-layout-json-format)
10. [Module reference](#module-reference)

---

## Architecture overview

```
                      +---------------------------------------------+
  Camera              |  Jetson Orin Nano Super                      |
  (MIPI / USB)        |                                              |
      |               |  +------------------------------------------+|
      +---------------+->|  isaac_ros_apriltag  (CUDA/GPU)           ||
                      |  |  /apriltag/detection_array                ||
                      |  +--------------------+---------------------+|
                      |                       | AprilTagDetectionArray|
                      |  +--------------------v---------------------+|
                      |  |  vision_node  (this package)              ||
                      |  |                                           ||
                      |  |  * convertDetection()                     ||
                      |  |    - Isaac ROS GPU pose (default)         ||
                      |  |    - optional CPU solvePnP refinement     ||
                      |  |    - optical -> WPILib camera frame       ||
                      |  |  * estimateMultiTag()                     ||
                      |  |    - multi-point PnP (SQPNP)              ||
                      |  |    - camera -> robot pose in field        ||
                      |  |  * NtPublisher                            ||
                      |  |    - NT4 -> RoboRIO                       ||
                      |  +------------------------------------------+|
                      +---------------------------------------------+
                                        |
                                        | NT4 (NetworkTables 4)
                                        v
                               +------------------+
                               |  RoboRIO          |
                               |  WPILib robot code|
                               +------------------+
```

### Module summary

| Module | File | Responsibility |
|--------|------|----------------|
| `coordinate_transforms` | `src/coordinate_transforms.cpp` | Frame conversions between camera-optical, WPILib camera (NWU), tag, and field frames |
| `field_layout` | `src/field_layout.cpp` | Load & validate WPILib-format field layout JSON |
| `pose_estimation` | `src/pose_estimation.cpp` | solvePnP single-tag (IPPE_SQUARE + ambiguity) and multi-tag (SQPNP) |
| `nt_publisher` | `src/nt_publisher.cpp` | Publish `PipelineResult` to NetworkTables 4 under `/Vision/` |
| `vision_node` | `src/vision_node.cpp` | ROS 2 node; subscribes to Isaac ROS CUDA detections, drives the full pipeline |

---

## Coordinate frame conventions

### Camera-optical frame (ROS / OpenCV)
```
        +z (into scene / forward)
       /
      /
     O----------> +x  (right in image)
     |
     v
    +y  (down in image)
```

### WPILib camera frame  (PhotonVision convention — NWU)
```
     +z (up)
     |
     |
     O----------> +y  (left in image)
    /
   /
 +x  (out of lens = forward)
```

Rotation from optical -> WPILib camera frame:
```
  PV_x =  optical_z
  PV_y = -optical_x
  PV_z = -optical_y
```

### WPILib robot / field frame  (NWU)
```
     +z (up)
     |
     |
     O----------> +y  (left / west)
    /
   /
 +x  (forward / north / down-field)
```

### AprilTag field frame  (stored in the field layout JSON)
```
     +z (up)
     |
     +------+
     | TAG  |----------> +x  (normal out of visible face)
     +------+
     |
     v
    +y  (right when looking at the tag)
```

---

## Hardware & software requirements

| Component | Minimum |
|-----------|---------|
| Jetson module | Orin Nano Super (or any Orin-series with CUDA 11.4+) |
| JetPack | 5.1.3 or 6.x |
| ROS 2 | Humble Hawksbill |
| CUDA | 11.4+ (bundled with JetPack) |
| Camera | Any v4l2, MIPI, or GigE camera supported by `image_pipeline` |

---

## Installing dependencies

### 1. ROS 2 Humble

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. NVIDIA Isaac ROS AprilTag  (**required**)

ROSVision uses Isaac ROS AprilTag exclusively as its detection backend because
it runs the AprilTag algorithm on the Jetson GPU/DLA for sub-millisecond
detection latency.

Full installation guide:
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/

Quick-start using the Isaac ROS Dev Docker (recommended):
```bash
# 1. Set up Isaac ROS common workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 2. Clone the AprilTag package
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# 3. Launch the Isaac ROS Dev Docker (provides CUDA, cuDNN, TensorRT)
cd ~/ros2_ws/src/isaac_ros_common
./scripts/run_dev.sh

# --- Inside Docker ---
# 4. Install rosdeps
cd /workspaces/isaac_ros-dev
rosdep install -i -r --from-paths src --rosdistro humble -y

# 5. Build
colcon build --symlink-install --packages-up-to isaac_ros_apriltag
source install/setup.bash
```

> **Note:** The Isaac ROS AprilTag node requires a calibrated camera and
> publishes `AprilTagDetectionArray` on `/apriltag/detection_array`.
> Ensure your camera driver is publishing `/image_rect` and `/camera_info`.

### 3. WPILib ntcore  (required for live NT publishing)

Without ntcore the node still builds and the NT publisher prints results to
stdout (useful for offline testing).

**Option A — build from source on Jetson (aarch64):**
```bash
cd ~
git clone https://github.com/wpilibsuite/allwpilib.git
cd allwpilib
cmake -B build \
    -DWITH_JAVA=OFF \
    -DWITH_CSCORE=OFF \
    -DWITH_NTCORE=ON \
    -DWITH_WPIMATH=OFF \
    -DWITH_GUI=OFF \
    -DWITH_SIMULATION_MODULES=OFF \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build --target ntcore -j$(nproc)
sudo cmake --install build
# ntcoreConfig.cmake installed to /usr/local/lib/cmake/ntcore/
```

**Option B — pre-built x86-64 (desktop testing only):**
Download from https://github.com/wpilibsuite/allwpilib/releases and run the installer.

### 4. Other system libraries

```bash
sudo apt install -y \
    libeigen3-dev \
    libopencv-dev \
    nlohmann-json3-dev   # optional — CMake fetches it automatically if absent
```

---

## Build

Clone ROSVision into your Isaac ROS workspace and build with colcon:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Antigro09/ROSVision.git

# Inside Isaac ROS Dev Docker (or with ROS 2 sourced):
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros_vision
source install/setup.bash
```

If ntcore was installed to `/usr/local`, add it to the CMake prefix path:
```bash
colcon build \
    --symlink-install \
    --packages-select ros_vision \
    --cmake-args "-DCMAKE_PREFIX_PATH=/usr/local;/opt/ros/humble"
```

---

## Configuration

Copy and edit the sample parameter file:
```bash
cp $(ros2 pkg prefix ros_vision)/share/ros_vision/config/camera_config.yaml \
   ~/my_camera_config.yaml
```

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `field_layout_path` | `""` | Absolute path to WPILib field layout JSON |
| `tag_size_metres` | `0.1651` | Tag side length in metres (2024 FRC = 6.5 in) |
| `detection_topic` | `/apriltag/detection_array` | Isaac ROS AprilTag output topic |
| `use_isaac_single_tag_pose` | `true` | Use Isaac ROS GPU single-tag pose directly; set `false` to force local CPU solvePnP refinement |
| `cam_robot.x/y/z` | `0` | Camera→robot translation (metres) |
| `cam_robot.roll/pitch/yaw` | `0` | Camera→robot rotation (degrees) |
| `nt.team_number` | `0` | WPILib team number (0 = NT server mode) |
| `nt.server_address` | `""` | Explicit NT server IP (overrides team number) |
| `nt.root_table` | `/Vision` | NT root table path |

---

## Running

### 1. Start the Isaac ROS AprilTag CUDA node

```bash
ros2 run isaac_ros_apriltag isaac_ros_apriltag_node \
    --ros-args \
    -p tag_family:=36h11 \
    -p size:=0.1651 \
    -p max_tags:=32
```

> Make sure `/image_rect` and `/camera_info` are being published.
> Use `isaac_ros_argus_camera` or a v4l2 driver + `image_proc` to produce them.

### 2. Start the ROSVision node

```bash
ros2 run ros_vision vision_node \
    --ros-args \
    --params-file ~/my_camera_config.yaml
```

### 3. Competition-day camera calibration GUI (PhotonVision-style dashboard)

Use the built-in styled dashboard to tune camera + pipeline settings quickly on
competition days.

```bash
ros2 run ros_vision camera_calibration_gui -- --device 0
```

Dashboard includes:
- Cameras header with FPS and latency.
- Processed stream view panel.
- Camera panel (team number, camera nickname, camera type, resolution, stream resolution).
- Pipeline panel (AprilTag type, processing mode, decimate, multitag, orientation, low-latency mode).
- Camera controls: auto exposure, exposure, brightness, camera gain, red/blue AWB gain, auto white balance, white balance temperature.
- 3D mode output table fields:
  - ID
  - X-meters
  - Y-meters
  - Z-angle theta
  - ambiguity ratio

Controls:
- Move sliders to tune settings in real time.
- Press `n` to rename camera nickname.
- Press `s` to export a YAML snapshot.
- Press `q` or `Esc` to quit.

Persistence behavior:
- Every change is auto-saved immediately to:
  `~/.config/rosvision/camera_calibration_profile.json`
- On next launch, settings are auto-loaded for that camera device.
- Because settings are saved on disk, they persist across full device reboots.

Optional overrides:
```bash
ros2 run ros_vision camera_calibration_gui -- \
    --device /dev/video0 \
    --team-number 1234 \
    --camera-nickname FrontCam \
    --save-path ~/camera_calibration.yaml \
    --profile-path ~/.config/rosvision/camera_calibration_profile.json
```

Recommended AprilTag competition defaults (OV9281 example):
- Resolution: `1280x800`
- Decimate: `2`
- Processing Mode: `3D`
- MultiTag: enabled
- Camera Type: `OV9281`

### 4. One-shot with inline parameters

```bash
ros2 run ros_vision vision_node --ros-args \
    -p field_layout_path:=$(ros2 pkg prefix ros_vision)/share/ros_vision/config/field_layout.json \
    -p tag_size_metres:=0.1651 \
    -p nt.team_number:=1234
```

### Verify NT output

```bash
# Using Python ntcore client
python3 - <<'EOF'
import ntcore, time
inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("verify")
inst.setServerTeam(1234)
time.sleep(2.0)
t = inst.getTable("/Vision")
print("hasTarget :", t.getEntry("hasTarget").getBoolean(False))
print("numTargets:", t.getEntry("numTargets").getInteger(0))
mt = t.getSubTable("multiTag")
print("robotX    :", mt.getEntry("robotX").getDouble(-1.0))
print("robotY    :", mt.getEntry("robotY").getDouble(-1.0))
print("robotYaw  :", mt.getEntry("robotYaw").getDouble(-1.0))
EOF
```

---

## NetworkTables schema

All entries are published under the root table (default `/Vision`).

### Main table  `/Vision/`

| Entry | Type | Description |
|-------|------|-------------|
| `timestamp` | double | Capture timestamp in seconds (FPGA epoch) |
| `latencyMs` | double | End-to-end latency in milliseconds |
| `hasTarget` | boolean | `true` if any tag was detected |
| `numTargets` | int | Number of detected tags |
| `targetYaw` | double | Yaw to best target in degrees (+left) |
| `targetPitch` | double | Pitch to best target in degrees (+up) |
| `targetSkew` | double | Best target rotation in degrees |
| `targetArea` | double | Bounding-box area as % of image (stub) |
| `pipelineResultJson` | string | Full result serialised as JSON |

### Multi-tag sub-table  `/Vision/multiTag/`

| Entry | Type | Description |
|-------|------|-------------|
| `robotX` | double | Robot X in field frame (metres) |
| `robotY` | double | Robot Y in field frame (metres) |
| `robotZ` | double | Robot Z in field frame (metres) |
| `robotYaw` | double | Robot yaw in field frame (degrees) |
| `reprojErr` | double | Mean multi-tag reprojection error (pixels) |
| `numTags` | int | Number of tags used in the solve |

---

## Field layout JSON format

```json
{
  "field": { "length": 16.5412, "width": 8.2118 },
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": { "x": 15.0798, "y": 0.2458, "z": 1.3557 },
        "rotation": {
          "quaternion": { "W": 0.5, "X": 0.0, "Y": 0.0, "Z": 0.866 }
        }
      }
    }
  ]
}
```

- `field.length` — field length in metres along the X axis.
- `field.width`  — field width in metres along the Y axis.
- `tags[].ID`    — integer AprilTag ID (must be unique and non-negative).
- `tags[].pose`  — tag pose in the WPILib NWU field frame.
  - `translation` — in metres.
  - `rotation.quaternion` — normalised `{W, X, Y, Z}`.
- The tag +x axis points normal out of the visible face of the tag.
- A sample 2024 Crescendo layout with all 16 tags is at `config/field_layout.json`.

---

## Module reference

### `coordinate_transforms`
Implements `opticalToPhotonCamera()`, `opticalToRobotFrame()`,
`cameraPoseToRobotPose()`, and `solvePnPToTransform()`.
Full frame diagrams and sign conventions are documented in
`include/ros_vision/coordinate_transforms.hpp`.

### `field_layout`
`loadFieldLayout(path)` and `parseFieldLayout(json_string)` — both throw
`std::runtime_error` with a descriptive message on any schema or validation
error.  Required keys, numeric ranges, and quaternion norm are all validated.

### `pose_estimation`
`PoseEstimator::estimateSingleTag()` — runs `cv::SOLVEPNP_IPPE_SQUARE` to get
both pose solutions, computes reprojection error for each, and derives the
ambiguity ratio (best_error / alt_error in [0, 1]).

By default, `vision_node` uses Isaac ROS AprilTag's GPU-provided single-tag pose
(`use_isaac_single_tag_pose=true`) for lower CPU load; set the parameter to
`false` to re-enable local `estimateSingleTag()` refinement. In GPU-pose mode,
single-tag `bestReprojError` and `ambiguity` are published as `-1.0` (not
computed), trading those diagnostics for lower per-frame CPU work.

`PoseEstimator::estimateMultiTag()` — collects 3-D tag corner positions from
the field layout for every detected tag ID, then runs `cv::SOLVEPNP_SQPNP`
across all correspondences for a single robust camera/robot field-pose estimate.
Returns a `MultiTagResult` with `isValid=false` when fewer than 2 layout-matched
tags are visible.

### `nt_publisher`
Wraps `nt::NetworkTableInstance`.  When built without ntcore (`HAVE_NTCORE` not
defined), falls back to printing `[NT-STUB]` lines to stdout so the full
pipeline can be exercised without WPILib installed.

---

## License

MIT
