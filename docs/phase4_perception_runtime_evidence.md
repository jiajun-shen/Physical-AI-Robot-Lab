# Phase-4 RGB-D Perception Runtime Evidence

## P4.1 camera transport and frame contract

Status: `PASS` for the P4.1 transport subtask; Phase 4 as a whole remains
`PARTIAL`.

- Tested commit: `feca6ba` (`feat(phase4): add isolated RGB-D camera transport`)
- Date: 2026-08-14
- Environment: WSL Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11.0
- Mode: `PERCEPTION_MODE=vision` is the required next pipeline mode; no
  perception node or model-pose input was started in this transport run.
- Build/test root: `/var/tmp/physical-ai-p4p1.vkHkFS`
- Runtime evidence root: `/var/tmp/physical-ai-p4p1-runtime.CY69QB`

The clean-commit targeted build completed `abb_lab_description`,
`abb_lab_lab`, `abb_lab_gripper`, and `abb_lab_bringup`. The related source
contracts reported 18 tests with zero errors, failures, or skips. The
dedicated P4 launch reused the existing gripper/lab launch and its sole
Gazebo-hosted controller manager; it added no second control chain.

The real headless run observed all three one-way ROS sensor topics:

```text
/lab_camera/color/image_raw       sensor_msgs/msg/Image
/lab_camera/depth/image_raw       sensor_msgs/msg/Image
/lab_camera/color/camera_info     sensor_msgs/msg/CameraInfo
```

The captured CameraInfo reports `width=640`, `height=480`, a finite positive
K matrix (`fx=fy=457.007362155546...`), and
`frame_id=lab_camera_optical_frame`. `tf2_echo` resolved
`world -> lab_camera_optical_frame` from the two static transforms, using the
same camera pose as the independent RGB-D SDF world.

This result proves sensor transport and the camera-frame chain only. It does
not yet prove RGB inspection, depth encoding/invalid-value handling,
CameraInfo projection, object detection, world localization, offline
ground-truth comparison, or A03. No Gazebo pose was bridged into ROS or used
as a perception input.

## Next exact subtask

Implement the project-owned typed messages and vision-only RGB/depth/
CameraInfo/TF2 pipeline, then save sealed vision evidence before running the
separate offline Gazebo comparison for seeds 42 and 43.
