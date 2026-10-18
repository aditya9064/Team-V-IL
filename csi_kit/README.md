# CSI visit kit (Sat Oct 17)

## Real-photo test (works offline on this Mac)

1. Copy frames from the cell camera into `csi_kit/photos/` (jpg or png, full uncropped frame).
2. Run in Terminal:
   - table camera: `bash ~/Desktop/AdaptiveVision/csi_kit/run_test.sh`
   - side camera:  `bash ~/Desktop/AdaptiveVision/csi_kit/run_test.sh side`
3. A results folder opens. For each photo:
   - `*_overlay.jpg`: every object tagged #n. Green = cube ready to pick, grey = not in the tower task, red = cube rejected (reason shown).
   - `*_picklist.csv`: ordered picks (x, y in mm in the robot base frame, tip height in base and UF0 frames, cube yaw, place position, stack level). For MENTOR REVIEW only.
   - `*_result.json`: everything, including timing.
4. Delete `photos/example_sim_1080p.jpg` first: it is a simulator frame used to test the kit.

Any resolution with the camera's 16:9 shape works (1080p is scaled to the calibrated 4K). Cropped or rotated
photos are refused because the calibration would not apply.

## Safety rules (event rules)

- Do not move or re-aim any camera. Do not change the PLC or robot controller.
- No commands to the physical arm without the plant manager AND the assigned mentor.
- The pick list is information for the mentor; nothing in this kit talks to the robot.

## Questions to ask (in this order)

1. Cube size: 10 mm (starter task) or 16 mm (CAD model)? Measure one. The gripper opens 12 mm.
2. Which cube colours physically exist? (Website photos show yellow only.)
3. Real task: the starter's 6-cube tower, or sorting into bins?
4. Gripper: SMC MHZL2-16D2 or SCHUNK?
5. Table camera native resolution (calibration is 1920x1080; we render 3840x2160 with the same view).
6. Have any cameras moved since the v0.5 ArUco calibration? Is there a newer calibration file?
7. How is the arm commanded: teach pendant, KAREL, RMI, ROS? Can we get a supervised slot to run our pick list?
8. May we save 40-60 frames (table / side / wrist) for testing only (not training)?

## Photos to take (with permission)

- 10+ table-camera frames: cubes scattered, touching, near caps/vials, different lighting if possible.
- 10 side-camera and 10 wrist-camera frames if allowed.
- A phone photo of the whole cell and one of a cube next to a ruler (for size).

## What to show the mentor

- Robot video page (private, share from its menu if needed): https://claude.ai/artifact/MKnKMSUASwLrNZejAq96DE
- Simulator baseline (YOLO v1, 20 random layouts): 20/20 towers, 120/120 picks and placements,
  0.21 mm median position error, 1.35 deg yaw error, 55 s per tower.
