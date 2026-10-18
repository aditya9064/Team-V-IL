"""
V2 synthetic data: the hard cases (towers, touching cubes, arm in view,
distractors, look-alike tape, textured mats, shadows, side/wrist cameras,
sensor effects) with occlusion-aware labels from the rendered depth.

    "$ISAAC/python.sh" synth/generate_v2.py --out OUT --camera table|side|wrist --start N --count K

One camera per run (each extra camera would render every step). Frames are
deterministic per index and existing frames are skipped, so runs resume.
Labels come from authored poses (twin ground truth) and are training targets
only. Static scene logic is unit-tested in synth/test_v2.py.
"""

import argparse, dataclasses, json, os, sys, time

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--camera", required=True, choices=["table", "side", "wrist"])
parser.add_argument("--start", type=int, default=0)
parser.add_argument("--count", type=int, default=20)
parser.add_argument("--seed", type=int, default=20261016)
parser.add_argument("--settle", type=int, default=6)
parser.add_argument("--prefix", default="v2_")
parser.add_argument("--gpu", type=int, default=0, help="GPU index inside a multi-GPU job")
parser.add_argument("--renderer", default="realtime", choices=["realtime", "pathtracing"],
                    help="pathtracing = OptiX-denoised path tracing (needs a fast RTX GPU)")
parser.add_argument("--spp", type=int, default=64, help="path-tracing samples per pixel")
parser.add_argument("--denoise", default="median3", choices=["median3", "none"],
                    help="median3 hides real-time speckle on GPUs without DLSS (e.g. T4)")
args = parser.parse_args()
args.out = os.path.abspath(args.out)

from isaacsim import SimulationApp

# Pin rendering and physics to one GPU so several copies can share a multi-GPU job.
sim_app = SimulationApp({"headless": True, "active_gpu": args.gpu, "physics_gpu": args.gpu,
                         "multi_gpu": False})

import cv2
import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
from isaacsim.core.api import World
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import scene_config as C, cell
from backproject import CameraModel
from fk import fk
from synth import labels, mat_textures, scenarios, sensor_fx

SURFACE_Z = C.TABLE_TOP_Z + C.SURFACE_DZ
PARK = Gf.Vec3d(-0.30, 0.60, SURFACE_Z)  # off the mat, outside every view
CAMS = {"table": (cell.TABLE_CAM_PATH, C.TABLE_CAM, 2),  # (prim, config, resolution scale)
        "side": (cell.SIDE_CAM_PATH, C.SIDE_CAM, 1),
        "wrist": (cell.WRIST_CAM_PATH, C.WRIST_CAM, 1)}
GEOMS = {"cube": UsdGeom.Cube, "sphere": UsdGeom.Sphere, "cylinder": UsdGeom.Cylinder,
         "cone": UsdGeom.Cone, "capsule": UsdGeom.Capsule}
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")


# ---------------------------------------------------------------- scene build
def ops(prim):
    return {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}


def op_of(prim, kind):
    return next(o for o in UsdGeom.Xformable(prim).GetOrderedXformOps() if o.GetOpType() == kind)


def make_objects_static(stage):
    """Physics drives only the arm; every object is posed directly in USD."""
    for prim in stage.Traverse():
        if prim.GetName().startswith(("SortCube", "SortVial", "SortCap")) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)


def add_extra_cubes(stage):
    """cell.py builds six cubes; v2 scenes use up to twelve (visual only)."""
    for i in range(len(C.SORT_CUBES), scenarios.CUBE_SLOTS):
        xf = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, f"/World/SortCube{i}").GetPrim())
        xf.AddTranslateOp().Set(PARK)
        xf.AddRotateZOp().Set(0.0)
        vis = f"/World/SortCube{i}/Visual"
        cell.add_reference_to_stage(usd_path=os.path.join(HERE, "assets/usd_assets/cube.usda"), prim_path=vis)
        cell.scale_cad_visual(stage, vis, (C.CUBE_EDGE / 0.016,) * 3)


def add_primitive(stage, path, shape):
    geom = GEOMS[shape].Define(stage, path)
    if shape == "cube":
        geom.CreateSizeAttr(1.0)
    elif shape == "sphere":
        geom.CreateRadiusAttr(0.5)
    elif shape == "capsule":
        geom.CreateRadiusAttr(0.25)
        geom.CreateHeightAttr(0.5)
    else:
        geom.CreateRadiusAttr(0.5)
        geom.CreateHeightAttr(1.0)
    if shape != "cube" and shape != "sphere":
        geom.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(geom.GetPrim())
    xf.AddTranslateOp().Set(PARK)
    xf.AddRotateZOp().Set(0.0)
    xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()


def add_distractors(stage):
    UsdGeom.Scope.Define(stage, "/World/V2")
    for shape, n in scenarios.DISTRACTOR_SLOTS.items():
        for k in range(n):
            add_primitive(stage, f"/World/V2/Distractor_{shape}{k}", shape)
    for k in range(scenarios.TAPE_SLOTS):
        add_primitive(stage, f"/World/V2/Tape{k}", "cube")


def palette_materials():
    for name, rgb, _ in scenarios.PALETTE:
        OmniPBR(prim_path=f"/World/Looks/v2_{name}", name=f"v2_{name}", color=np.array(rgb, float))


def add_mat_overlay(stage, texture_paths):
    """A quad just above the mat; hidden when the frame keeps the stock mat."""
    (x0, x1), (y0, y1), z = C.TABLE_X, C.TABLE_Y, SURFACE_Z + 0.0002
    mesh = UsdGeom.Mesh.Define(stage, "/World/V2/MatOverlay")
    mesh.CreatePointsAttr([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    reps = 4.0
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.varying).Set(
        [(0, 0), (reps, 0), (reps, reps), (0, reps)])
    for i, path in enumerate(texture_paths):
        OmniPBR(prim_path=f"/World/Looks/v2_mat{i}", name=f"v2_mat{i}", texture_path=path,
                color=np.array([1.0, 1.0, 1.0]))
    return OmniPBR(prim_path="/World/Looks/v2_mat_solid", name="v2_mat_solid", color=np.array([0.8, 0.8, 0.8]))


def bind(stage, prim_path, material_path):
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(prim_path)).Bind(
        UsdShade.Material(stage.GetPrimAtPath(material_path)), UsdShade.Tokens.strongerThanDescendants)


# ---------------------------------------------------------------- per frame
def show(prim, visible):
    image = UsdGeom.Imageable(prim)
    image.MakeVisible() if visible else image.MakeInvisible()
    return visible


def pose_cube(stage, item):
    prim = stage.GetPrimAtPath(f"/World/{item.name}")
    translate = op_of(prim, UsdGeom.XformOp.TypeTranslate)
    if not show(prim, item.visible):
        translate.Set(PARK)
        return
    translate.Set(Gf.Vec3d(item.x, item.y, SURFACE_Z + item.z))
    op_of(prim, UsdGeom.XformOp.TypeRotateZ).Set(float(item.yaw_deg))
    visual = stage.GetPrimAtPath(f"/World/{item.name}/Visual")
    visual.GetVariantSets().GetVariantSet("color").SetVariantSelection(item.colour)


def pose_cap(stage, item):
    prim = stage.GetPrimAtPath(f"/World/{item.name}")
    translate = op_of(prim, UsdGeom.XformOp.TypeTranslate)
    if not show(prim, item.visible):
        translate.Set(PARK)
        return
    translate.Set(Gf.Vec3d(item.x, item.y, SURFACE_Z))
    vis = ops(stage.GetPrimAtPath(f"/World/{item.name}/Visual"))
    narrow = item.variant == "narrow_up"  # same transforms cell.py authors per orientation
    vis["xformOp:translate:rimRest"].Set(Gf.Vec3d(0, 0, C.CAP_FLANGE_THICK if narrow else C.CAP_PLUG_LENGTH))
    vis["xformOp:rotateX:domeUp"].Set(180.0 if narrow else 0.0)


def pose_vial(stage, item):
    prim = stage.GetPrimAtPath(f"/World/{item.name}")
    translate = op_of(prim, UsdGeom.XformOp.TypeTranslate)
    if not show(prim, item.visible):
        translate.Set(PARK)
        return
    yaw = np.deg2rad(item.yaw_deg)
    op_of(prim, UsdGeom.XformOp.TypeRotateZ).Set(float(item.yaw_deg))
    if item.variant == "upright":  # local +Z (tube axis) stays vertical
        op_of(prim, UsdGeom.XformOp.TypeRotateY).Set(0.0)
        translate.Set(Gf.Vec3d(item.x, item.y, SURFACE_Z))
        return
    op_of(prim, UsdGeom.XformOp.TypeRotateY).Set(90.0)
    tail = C.VIAL_LENGTH / 2 * np.array([np.cos(yaw), np.sin(yaw)])
    translate.Set(Gf.Vec3d(item.x - tail[0], item.y - tail[1], SURFACE_Z + C.VIAL_RADIUS + 0.001))


def pose_extra(stage, item):
    """Distractor primitives and flat tape; never labelled."""
    path = f"/World/V2/{item.name}"
    prim = stage.GetPrimAtPath(path)
    o = ops(prim)
    if not show(prim, item.visible):
        o["xformOp:translate"].Set(PARK)
        return
    if item.category == "tape":
        o["xformOp:scale"].Set(Gf.Vec3f(item.size, 0.4 * item.size, 0.0005))
        z = SURFACE_Z + 0.00025
    else:
        o["xformOp:scale"].Set(Gf.Vec3f(item.size, item.size, item.size))
        z = SURFACE_Z + item.size / 2
    o["xformOp:translate"].Set(Gf.Vec3d(item.x, item.y, z))
    o["xformOp:rotateZ"].Set(float(item.yaw_deg))
    bind(stage, path, f"/World/Looks/v2_{item.colour}")


def pose_scene(stage, scene):
    posers = {"cube": pose_cube, "cap": pose_cap, "vial": pose_vial}
    for item in scene.objects:
        posers[item.category](stage, item)
    for item in scene.distractors:
        pose_extra(stage, item)


def read_lights(stage):
    base = {}
    for name in scenarios.LIGHT_NAMES:
        prim = stage.GetPrimAtPath(f"/World/{name}")
        light = UsdLux.LightAPI(prim)
        rot = [o for o in UsdGeom.Xformable(prim).GetOrderedXformOps()
               if o.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ]
        base[name] = (light.GetIntensityAttr().Get(), light.GetColorAttr().Get(),
                      rot[0] if rot else None, rot[0].Get() if rot else None)
    return base


def apply_lights(stage, scene, base):
    for name, scale, tint, jitter in scene.lights:
        light = UsdLux.LightAPI(stage.GetPrimAtPath(f"/World/{name}"))
        intensity, colour, rot_op, rot0 = base[name]
        light.GetIntensityAttr().Set(intensity * scale)
        colour = colour if colour is not None else Gf.Vec3f(1, 1, 1)
        light.CreateColorAttr().Set(Gf.Vec3f(*(c * t for c, t in zip(colour, tint))))
        if rot_op is not None:
            rot_op.Set(Gf.Vec3f(*(r + j for r, j in zip(rot0, jitter))))


def apply_mat(stage, scene, solid):
    index, tint = scene.mat
    overlay = stage.GetPrimAtPath("/World/V2/MatOverlay")
    if not show(overlay, index != -1):
        return
    if index == -2:
        solid.set_color(np.array(tint, float))
        bind(stage, "/World/V2/MatOverlay", "/World/Looks/v2_mat_solid")
    else:
        bind(stage, "/World/V2/MatOverlay", f"/World/Looks/v2_mat{index}")


def set_arm(arm, names, pose):
    q = arm.get_joint_positions()
    for joint in JOINTS:
        q[names.index(joint)] = np.deg2rad(pose[joint])
    for joint, value in cell.jaw_targets(pose["jaw"]).items():
        q[names.index(joint)] = value
    arm.set_joint_positions(q)
    arm.apply_action(ArticulationAction(joint_positions=q))


# ---------------------------------------------------------------- camera
def look_at_rows(eye, target, up):
    """USD row-vector camera-to-world rows: right, up, back (camera looks down -Z)."""
    axis = np.asarray(target, float) - eye
    axis /= np.linalg.norm(axis)
    right = np.cross(axis, up)
    right /= np.linalg.norm(right)
    return right, np.cross(right, axis), -axis


def place_fixed_camera(cam, cfg, scene):
    eye = np.add(cfg["eye"], scene.camera_eye_offset)
    target = np.add(cfg["look_at"], scene.camera_target_offset)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*cfg["up"])).GetInverse()
    q = view.ExtractRotationQuat()
    cam.set_world_pose(position=eye, orientation=np.array([q.GetReal(), *q.GetImaginary()]),
                       camera_axes="usd")


def wrist_rows(arm, names):
    """Wrist camera pose from FK of the joints actually reached (not a USD readback)."""
    q = arm.get_joint_positions()
    R, p = fk([float(q[names.index(j)]) for j in JOINTS])
    cfg = C.WRIST_CAM
    right, up, back = look_at_rows(np.array(cfg["eye"]), cfg["look_at"], cfg["up"])
    base = np.array([0.0, 0.0, C.TABLE_TOP_Z])
    eye = base + p + R @ np.array(cfg["eye"])
    return [R @ right, R @ up, R @ back], eye


def camera_json(stage, path, resolution, rows_eye=None):
    gcam = UsdGeom.Camera(stage.GetPrimAtPath(path))
    if rows_eye is None:
        xf = UsdGeom.Xformable(gcam.GetPrim()).ComputeLocalToWorldTransform(0)
        matrix = [[float(xf[r][c]) for c in range(4)] for r in range(4)]
    else:
        rows, eye = rows_eye
        matrix = [list(map(float, rows[0])) + [0.0], list(map(float, rows[1])) + [0.0],
                  list(map(float, rows[2])) + [0.0], list(map(float, eye)) + [1.0]]
    return {"resolution": list(resolution), "focal_length_mm": float(gcam.GetFocalLengthAttr().Get()),
            "horizontal_aperture_mm": float(gcam.GetHorizontalApertureAttr().Get()),
            "vertical_aperture_mm": float(gcam.GetVerticalApertureAttr().Get()),
            "camera_to_world": matrix, "table_top_z": float(SURFACE_Z)}


def render(world, cam, settle, resolution, max_wait=600):
    """Step until colour and depth arrive (fresh nodes compile shaders for minutes)."""
    for _ in range(settle):
        world.step(render=True)
    for _ in range(max_wait):
        rgba, depth = cam.get_rgba(), cam.get_depth()
        if rgba is not None and rgba.size and depth is not None and depth.size:
            if rgba.shape[1::-1] != resolution or depth.shape[:2] != rgba.shape[:2]:
                raise RuntimeError(f"Rendered {rgba.shape}/{depth.shape}, expected {resolution}")
            return rgba[:, :, :3].astype(np.uint8), depth.astype(np.float32)
        world.step(render=True)
    raise RuntimeError(f"No colour/depth after {max_wait} extra frames; check the GPU")


# ---------------------------------------------------------------- output + main
def write_frame(stem, rgb, roi, lines, vis, scene, cam_json):
    x0, y0, x1, y1 = roi
    crop = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2BGR)
    if args.denoise == "median3":
        crop = cv2.medianBlur(crop, 3)  # real-time renderer speckle (no DLSS); see v1 notes
    crop = sensor_fx.apply(crop, scene.sensor, scene.seed)
    cv2.imwrite(os.path.join(args.out, "images", f"{stem}.png"), crop)
    with open(os.path.join(args.out, "labels", f"{stem}.txt"), "w") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))
    meta = {"scene": dataclasses.asdict(scene), "camera": cam_json, "roi": list(roi), "visibility": vis}
    with open(os.path.join(args.out, "meta", f"{stem}.json"), "w") as f:
        json.dump(meta, f)


def setup():
    if args.renderer == "pathtracing":
        import render_settings

        render_settings.enable_path_tracing(spp=args.spp)
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    cell.build(stage)
    make_objects_static(stage)
    add_extra_cubes(stage)
    add_distractors(stage)
    palette_materials()
    textures = mat_textures.make_textures(os.path.join(args.out, "_mat_textures"), seed=args.seed)
    solid = add_mat_overlay(stage, textures)
    path, cfg, scale = CAMS[args.camera]
    resolution = tuple(scale * v for v in cfg["resolution"])
    cam = Camera(prim_path=path, resolution=resolution)
    world.reset()
    cam.initialize()
    cam.add_distance_to_image_plane_to_frame()
    arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="er4ia")
    arm.initialize()
    for _ in range(90):  # MDL materials compile over the first frames
        world.step(render=True)
    return world, stage, cam, arm, solid, resolution


def frame_camera(stage, cam, arm, names, scene, resolution, focal0):
    path, cfg, _ = CAMS[args.camera]
    UsdGeom.Camera(stage.GetPrimAtPath(path)).GetFocalLengthAttr().Set(focal0 * scene.focal_scale)
    if args.camera == "wrist":
        return lambda: camera_json(stage, path, resolution, wrist_rows(arm, names))
    place_fixed_camera(cam, cfg, scene)
    return lambda: camera_json(stage, path, resolution)


def main():
    for sub in ("images", "labels", "meta"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    world, stage, cam, arm, solid, resolution = setup()
    names = list(arm.dof_names)
    path = CAMS[args.camera][0]
    focal0 = UsdGeom.Camera(stage.GetPrimAtPath(path)).GetFocalLengthAttr().Get()
    cfg = scenarios.V2Config()
    roi = (labels.workspace_roi(CameraModel(camera_json(stage, path, resolution)), cfg.x_range, cfg.y_range)
           if args.camera == "table" else (0, 0, *resolution))
    base_lights = read_lights(stage)
    manifest = {"version": "v2", "classes": list(labels.CLASS_NAMES), "roi": list(roi),
                "camera": args.camera, "seed": args.seed, "prefix": args.prefix,
                "renderer": "rtx-pathtracing" if args.renderer == "pathtracing" else "rtx-realtime",
                "spp": args.spp if args.renderer == "pathtracing" else None,
                "denoise": args.denoise, "sensor_fx": True}
    with open(os.path.join(args.out, "dataset.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    t0 = time.time()
    for index in range(args.start, args.start + args.count):
        stem = f"{args.prefix}{index:06d}"
        if os.path.exists(os.path.join(args.out, "images", f"{stem}.png")):
            continue
        scene = scenarios.sample_v2(index, args.seed, args.camera, cfg)
        pose_scene(stage, scene)
        apply_lights(stage, scene, base_lights)
        apply_mat(stage, scene, solid)
        set_arm(arm, names, scene.arm)
        read_camera = frame_camera(stage, cam, arm, names, scene, resolution, focal0)
        rgb, depth = render(world, cam, args.settle, resolution)
        cam_json = read_camera()
        lines, vis = labels.label_lines_v2(scene, CameraModel(cam_json), roi, depth)
        write_frame(stem, rgb, roi, lines, vis, scene, cam_json)
        print(f"[synth] {stem}: {scene.camera}/{scene.mode} {len(lines)} labels, {time.time() - t0:.2f} s", flush=True)
        t0 = time.time()
    print(f"[synth] wrote frames {args.start}..{args.start + args.count - 1} to {args.out}")


# SimulationApp.close() ends the process, so report any exception before closing.
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
