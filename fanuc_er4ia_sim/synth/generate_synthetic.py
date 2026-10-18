"""
Render a domain-randomised, auto-labelled detection dataset from the twin.

Runs under Isaac's interpreter. Each frame re-poses the cubes, caps and vials,
re-colours cubes, rescales/tints the lights and jitters the table camera, then
saves the calibrated 4K table view cropped to the workspace, with YOLO labels.

    "$ISAAC/python.sh" synth/generate_synthetic.py --out synth_out/a --count 500
    "$ISAAC/python.sh" synth/generate_synthetic.py --out synth_out/b --start 500 --count 500

Frames are deterministic per index, so chunks can run on different machines
and an interrupted run resumes by skipping images that already exist.
Labels come from the poses this script sets, which is the twin's ground truth;
they are training targets only and never drive robot grasps.
"""

import argparse, dataclasses, json, os, sys, time

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--count", type=int, default=200)
parser.add_argument("--start", type=int, default=0)
parser.add_argument("--seed", type=int, default=20261015)
parser.add_argument("--settle", type=int, default=6, help="render frames per sample")
parser.add_argument("--pathtrace", action="store_true")
parser.add_argument("--spp", type=int, default=32)
parser.add_argument(
    "--denoise", choices=["median3", "none"], default="median3",
    help="post-render filter; without DLSS the real-time renderer leaves heavy speckle",
)
args = parser.parse_args()
# Kit changes the working directory; resolve before it starts.
args.out = os.path.abspath(args.out)

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import cv2
import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import scene_config as C, cell, render_settings
from backproject import CameraModel
from synth import labels, layout

SURFACE_Z = C.TABLE_TOP_Z + C.SURFACE_DZ
PARK_XY = (-0.30, 0.60)  # off the mat and outside the crop
OBJECT_PREFIXES = ("SortCube", "SortVial", "SortCap")


def make_objects_static(stage):
    """Physics drives only the arm; objects are posed directly in USD.

    Disabled rigid bodies are never written back by PhysX, so every authored
    pose is exactly what renders and what the labels describe.
    """
    for prim in stage.Traverse():
        if prim.GetName().startswith(OBJECT_PREFIXES) and prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)


def park_arm():
    """Same out-of-view pose as capture_scene.py."""
    arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="er4ia")
    arm.initialize()
    names = list(arm.dof_names)
    q = arm.get_joint_positions()
    for joint, deg in C.CAPTURE_POSE_DEG.items():
        q[names.index(joint)] = np.deg2rad(deg)
    for joint, value in cell.jaw_targets(C.JAW_OPEN).items():
        q[names.index(joint)] = value
    arm.set_joint_positions(q)
    arm.apply_action(ArticulationAction(joint_positions=q))


def xform_ops(prim):
    return {op.GetOpType(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}


def pose_object(stage, obj):
    prim = stage.GetPrimAtPath(f"/World/{obj.name}")
    image = UsdGeom.Imageable(prim)
    ops = xform_ops(prim)
    translate = ops[UsdGeom.XformOp.TypeTranslate]
    if not obj.visible:
        image.MakeInvisible()
        translate.Set(Gf.Vec3d(*PARK_XY, SURFACE_Z))
        return
    image.MakeVisible()
    if obj.category == "vial":
        yaw = np.deg2rad(obj.yaw_deg)
        tail = (C.VIAL_LENGTH / 2) * np.array([np.cos(yaw), np.sin(yaw)])
        z = SURFACE_Z + C.VIAL_RADIUS + 0.001
        translate.Set(Gf.Vec3d(obj.x - tail[0], obj.y - tail[1], z))
    else:
        translate.Set(Gf.Vec3d(obj.x, obj.y, SURFACE_Z))
    if obj.category in ("cube", "vial"):
        if UsdGeom.XformOp.TypeRotateZ not in ops:
            raise RuntimeError(f"{obj.name} has no rotateZ op; check cell.py layout")
        ops[UsdGeom.XformOp.TypeRotateZ].Set(float(obj.yaw_deg))
    if obj.category == "cube":
        visual = stage.GetPrimAtPath(f"/World/{obj.name}/Visual")
        visual.GetVariantSets().GetVariantSet("color").SetVariantSelection(obj.colour)


def read_lights(stage):
    base = {}
    for name in layout.LIGHT_NAMES:
        light = UsdLux.LightAPI(stage.GetPrimAtPath(f"/World/{name}"))
        base[name] = (light.GetIntensityAttr().Get(), light.GetColorAttr().Get())
    return base


def apply_lights(stage, sample, base):
    for ls in sample.lights:
        light = UsdLux.LightAPI(stage.GetPrimAtPath(f"/World/{ls.name}"))
        intensity, colour = base[ls.name]
        light.GetIntensityAttr().Set(intensity * ls.intensity_scale)
        tinted = colour if colour is not None else Gf.Vec3f(1.0, 1.0, 1.0)
        light.CreateColorAttr().Set(Gf.Vec3f(*(c * t for c, t in zip(tinted, ls.tint))))


def jitter_camera(cam, sample):
    """Move the table camera through the sensor API.

    Isaac's Camera re-authors the prim's xform ops on initialise, so the
    transform op cell.py wrote no longer exists; set_world_pose works with
    whatever ops the prim has. Labels use the pose read back afterwards.
    """
    cfg = C.TABLE_CAM
    eye = Gf.Vec3d(*(a + b for a, b in zip(cfg["eye"], sample.camera_eye_offset)))
    tgt = Gf.Vec3d(*(a + b for a, b in zip(cfg["look_at"], sample.camera_target_offset)))
    cam_to_world = Gf.Matrix4d().SetLookAt(eye, tgt, Gf.Vec3d(*cfg["up"])).GetInverse()
    q = cam_to_world.ExtractRotationQuat()
    cam.set_world_pose(
        position=np.array(eye), orientation=np.array([q.GetReal(), *q.GetImaginary()]),
        camera_axes="usd",
    )


def camera_json(stage, resolution):
    """Camera model read back from USD, as capture_scene.py records it."""
    prim = stage.GetPrimAtPath(cell.TABLE_CAM_PATH)
    gcam = UsdGeom.Camera(prim)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    return {
        "camera": "table",
        "resolution": list(resolution),
        "focal_length_mm": float(gcam.GetFocalLengthAttr().Get()),
        "horizontal_aperture_mm": float(gcam.GetHorizontalApertureAttr().Get()),
        "vertical_aperture_mm": float(gcam.GetVerticalApertureAttr().Get()),
        "camera_to_world": [[float(xf[r][c]) for c in range(4)] for r in range(4)],
        "table_top_z": float(SURFACE_Z),
    }


def render(world, cam, settle, resolution, max_wait=600):
    """Step until the camera delivers a full frame.

    On a node's first run Isaac compiles shaders for minutes and the annotator
    returns empty frames meanwhile, so wait for data instead of failing.
    """
    for _ in range(settle):
        world.step(render=True)
    for _ in range(max_wait):
        rgba = cam.get_rgba()
        if rgba is not None and rgba.size:
            if rgba.shape[1::-1] != resolution:
                raise RuntimeError(f"Rendered {rgba.shape[1::-1]}, expected {resolution}")
            return rgba[:, :, :3].astype(np.uint8)
        world.step(render=True)
    raise RuntimeError(f"Camera returned no image after {max_wait} extra frames; check the GPU")


def write_frame(stem, rgb, roi, lines, sample, cam_json):
    x0, y0, x1, y1 = roi
    crop = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2BGR)
    if args.denoise == "median3":
        # Measured on a T4 frame: speckle 11.0 -> 2.8 (path tracing: 4.5), 1 ms.
        # Pixel-local, so object boxes and labels are unchanged.
        crop = cv2.medianBlur(crop, 3)
    cv2.imwrite(os.path.join(args.out, "images", f"{stem}.png"), crop)
    with open(os.path.join(args.out, "labels", f"{stem}.txt"), "w") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))
    meta = {"sample": dataclasses.asdict(sample), "camera": cam_json, "roi": list(roi)}
    with open(os.path.join(args.out, "meta", f"{stem}.json"), "w") as f:
        json.dump(meta, f)


def setup():
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    cell.build(stage)
    make_objects_static(stage)
    if args.pathtrace:
        render_settings.enable_path_tracing(args.spp)
    resolution = tuple(2 * v for v in C.TABLE_CAM["resolution"])
    cam = Camera(prim_path=cell.TABLE_CAM_PATH, resolution=resolution)
    world.reset()
    cam.initialize()
    park_arm()
    for _ in range(90):  # MDL materials compile over the first frames
        world.step(render=True)
    return world, stage, cam, resolution


def main():
    for sub in ("images", "labels", "meta"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    world, stage, cam, resolution = setup()
    cfg = layout.Randomization()
    nominal = CameraModel(camera_json(stage, resolution))
    roi = labels.workspace_roi(nominal, cfg.x_range, cfg.y_range)
    base_lights = read_lights(stage)
    manifest = {
        "classes": list(labels.CLASS_NAMES),
        "roi": list(roi),
        "seed": args.seed,
        "randomization": dataclasses.asdict(cfg),
        "renderer": "pathtrace" if args.pathtrace else "rtx-realtime",
        "denoise": args.denoise,
    }
    with open(os.path.join(args.out, "dataset.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    t0 = time.time()
    for index in range(args.start, args.start + args.count):
        stem = f"{index:06d}"
        if os.path.exists(os.path.join(args.out, "images", f"{stem}.png")):
            continue
        sample = layout.sample_scene(index, args.seed, cfg)
        for obj in sample.objects:
            pose_object(stage, obj)
        apply_lights(stage, sample, base_lights)
        jitter_camera(cam, sample)
        rgb = render(world, cam, args.settle, resolution)
        cam_json = camera_json(stage, resolution)
        lines = labels.label_lines(sample, CameraModel(cam_json), roi)
        write_frame(stem, rgb, roi, lines, sample, cam_json)
        print(f"[synth] {stem}: {len(lines)} labels, {time.time() - t0:.2f} s", flush=True)
        t0 = time.time()
    print(f"[synth] wrote frames {args.start}..{args.start + args.count - 1} to {args.out}")


# SimulationApp.close() ends the process, so an exception raised in main() would
# vanish before Python printed it. Report it first, then close, then exit nonzero.
exit_code = 0
try:
    main()
except Exception:
    import traceback

    exit_code = 1
    message = traceback.format_exc()
    print(message, file=sys.stderr, flush=True)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "ERROR.txt"), "w") as f:
        f.write(message)
finally:
    sim_app.close()
    os._exit(exit_code)
