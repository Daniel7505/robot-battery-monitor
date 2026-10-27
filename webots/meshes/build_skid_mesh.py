# Headless Blender model for ButlerBotSkid.
# Run: blender --background --python webots/meshes/build_skid_mesh.py
#
# Coordinates match Webots ENU: X forward, Y left, Z up. OBJ is written
# from those coordinates. Webots does not swap OBJ axes (only Collada).
# Wheel mesh stays a Z-up cylinder; the PROTO rolls it 90 degrees about X.
# Sensor mounts are visual only and are left out of the collision boxes.

import json
import math
import shutil
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parent
PREVIEW = ROOT / "previews"
ART = Path("/opt/cursor/artifacts")

WHEEL_R = 0.08
WHEEL_H = 0.05
HUBS = (
    (0.11, 0.17, 0.08),
    (0.11, -0.17, 0.08),
    (-0.11, 0.17, 0.08),
    (-0.11, -0.17, 0.08),
)

# Device poses copied from ButlerBotSkid.proto. Do not move the devices.
SONARS = (
    ("us_front", (0.18, 0.0, 0.12), 0.0),
    ("us_front_left", (0.14, 0.16, 0.12), 0.7854),
    ("us_left", (0.0, 0.22, 0.12), 1.5708),
    ("us_rear_left", (-0.12, 0.16, 0.12), 2.3562),
    ("us_rear", (-0.18, 0.0, 0.12), math.pi),
    ("us_rear_right", (-0.12, -0.16, 0.12), -2.3562),
    ("us_right", (0.0, -0.22, 0.12), -1.5708),
    ("us_front_right", (0.14, -0.16, 0.12), -0.7854),
)
CORNER_SONARS = {
    "us_front_left",
    "us_front_right",
    "us_rear_left",
    "us_rear_right",
}


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


def material(name, color, metallic=0.25, rough=0.45, emit=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (color[0], color[1], color[2], 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    if emit > 0.0:
        color_key = "Emission Color" if "Emission Color" in bsdf.inputs else "Emission"
        bsdf.inputs[color_key].default_value = (color[0], color[1], color[2], 1.0)
        bsdf.inputs["Emission Strength"].default_value = emit
    return mat


def _finish(bm, name, col, mat, smooth=False):
    if smooth:
        for face in bm.faces:
            face.smooth = True
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    obj.data.materials.append(mat)
    col.objects.link(obj)
    return obj


def add_box(name, center, size, col, mat, quat=None):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    half = max(v.co.x for v in bm.verts)
    scale = [size[i] / (2.0 * half) for i in range(3)]
    rot = quat or Quaternion()
    origin = Vector(center)
    for vert in bm.verts:
        vert.co.x *= scale[0]
        vert.co.y *= scale[1]
        vert.co.z *= scale[2]
        vert.co = rot @ vert.co + origin
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _finish(bm, name, col, mat, smooth=False)


def add_cyl(name, center, radius, depth, col, mat, quat=None, segments=32):
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        cap_tris=True,
        segments=segments,
        radius1=radius,
        radius2=radius,
        depth=depth,
    )
    rot = quat or Quaternion()
    origin = Vector(center)
    for vert in bm.verts:
        vert.co = rot @ vert.co + origin
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _finish(bm, name, col, mat, smooth=True)


def add_sphere(name, center, radius, col, mat):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=16, radius=radius)
    origin = Vector(center)
    for vert in bm.verts:
        vert.co += origin
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _finish(bm, name, col, mat, smooth=True)


def quat_from_to(src, dst):
    return Vector(src).rotation_difference(Vector(dst))


def look_from_ypr(yaw=0.0, pitch=0.0):
    """Camera/radar default look is +X. Pitch about +Y sends it toward -Z."""
    forward = Vector((1.0, 0.0, 0.0))
    pitched = Quaternion((0.0, 1.0, 0.0), pitch) @ forward
    return Quaternion((0.0, 0.0, 1.0), yaw) @ pitched


def fmt(values):
    parts = []
    for value in values:
        if abs(value) < 5e-5:
            parts.append("0")
        else:
            parts.append(f"{value:.4f}".rstrip("0").rstrip("."))
    return " ".join(parts)


def world_mesh(obj):
    me = obj.data.copy()
    me.transform(obj.matrix_world)
    return me


def aabb_of(objects):
    xs, ys, zs = [], [], []
    for obj in objects:
        me = world_mesh(obj)
        for vert in me.vertices:
            xs.append(vert.co.x)
            ys.append(vert.co.y)
            zs.append(vert.co.z)
        bpy.data.meshes.remove(me)
    low = Vector((min(xs), min(ys), min(zs)))
    high = Vector((max(xs), max(ys), max(zs)))
    return (low + high) * 0.5, high - low


def write_obj(path, objects):
    lines = [
        "# ButlerBotSkid visual, meters",
        "# X forward, Y left, Z up (Webots ENU, raw OBJ, no axis swap)",
    ]
    offset = 0
    for obj in objects:
        me = world_mesh(obj)
        me.calc_loop_triangles()
        lines.append(f"o {obj.name}")
        for vert in me.vertices:
            co = vert.co
            lines.append(f"v {co.x:.6f} {co.y:.6f} {co.z:.6f}")
        for tri in me.loop_triangles:
            a, b, c = (index + 1 + offset for index in tri.vertices)
            lines.append(f"f {a} {b} {c}")
        offset += len(me.vertices)
        bpy.data.meshes.remove(me)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build():
    clear_scene()
    deck = collection("deck")
    upper = collection("upper")
    torso_col = collection("torso_collision")
    arm_l = collection("arm_l")
    arm_r = collection("arm_r")
    sensors = collection("sensors")
    lenses = collection("lenses")
    wheel_col = collection("wheel")
    hub_col = collection("hub")
    show = collection("show")

    mat_deck = material("deck", (0.16, 0.17, 0.19), metallic=0.45, rough=0.38)
    mat_ridge = material("ridge", (0.22, 0.24, 0.27), metallic=0.55, rough=0.32)
    mat_body = material("body", (0.18, 0.36, 0.52), metallic=0.2, rough=0.42)
    mat_head = material("head", (0.12, 0.13, 0.16), metallic=0.35, rough=0.3)
    mat_arm = material("arm", (0.45, 0.46, 0.48), metallic=0.4, rough=0.35)
    mat_mount = material("mount", (0.07, 0.075, 0.09), metallic=0.3, rough=0.4)
    mat_lens = material("lens", (0.15, 0.85, 1.0), metallic=0.0, rough=0.2, emit=8.0)
    mat_tire = material("tire", (0.015, 0.015, 0.018), metallic=0.05, rough=0.7)
    mat_hub = material("hub", (0.62, 0.63, 0.66), metallic=0.85, rough=0.25)
    mat_ground = material("ground", (0.55, 0.56, 0.58), metallic=0.0, rough=0.9)

    # Deck plate and a center ridge. Bottom stays 0.105 m, clear of the floor
    # and of the tire inner faces (|y| = 0.145).
    add_box("deck_plate", (0.0, 0.0, 0.125), (0.32, 0.24, 0.04), deck, mat_deck)
    add_box("deck_ridge", (0.0, 0.0, 0.153), (0.20, 0.14, 0.020), deck, mat_ridge)

    # Torso column sits on the deck. Neck and head are visual only, matching
    # the baseline (the head sphere was never a collider). GPS stays at the
    # head centre in the PROTO; this sphere is the shell around it.
    add_box("torso", (0.0, 0.0, 0.30), (0.16, 0.12, 0.28), torso_col, mat_body)
    add_box("shoulder", (0.0, 0.0, 0.40), (0.14, 0.10, 0.05), torso_col, mat_body)
    # Yokes are visual only. They bridge the torso shell to the arm boxes.
    add_box("yoke_l", (0.0, 0.085, 0.36), (0.07, 0.07, 0.08), upper, mat_body)
    add_box("yoke_r", (0.0, -0.085, 0.36), (0.07, 0.07, 0.08), upper, mat_body)
    add_cyl("neck", (0.0, 0.0, 0.50), 0.028, 0.12, upper, mat_head)
    add_sphere("head", (0.0, 0.0, 0.62), 0.060, upper, mat_head)
    eye_look = look_from_ypr(0.0, 0.15)
    add_cyl(
        "eye",
        Vector((0.0, 0.0, 0.62)) + eye_look * 0.055,
        0.012,
        0.008,
        lenses,
        mat_lens,
        quat=quat_from_to((0, 0, 1), eye_look),
        segments=16,
    )

    # Arms: same boxes the baseline used, hung from the torso joints at rest.
    add_box("arm_l", (0.0, 0.11, 0.21), (0.05, 0.05, 0.26), arm_l, mat_arm)
    add_box("arm_r", (0.0, -0.11, 0.21), (0.05, 0.05, 0.26), arm_r, mat_arm)

    # Nadir booms and boxes, same centres and rotations as the PROTO.
    boom_l = Quaternion((1.0, 0.0, 0.0), -0.767)
    boom_r = Quaternion((1.0, 0.0, 0.0), 0.767)
    add_cyl("nadir_boom_l", (0.00521, 0.26904, 0.57460), 0.012, 0.43, sensors, mat_mount, boom_l, 12)
    add_cyl("nadir_boom_r", (0.00521, -0.26904, 0.57460), 0.012, 0.43, sensors, mat_mount, boom_r, 12)
    add_box("nadir_box_l", (0.01042, 0.41808, 0.72919), (0.05, 0.05, 0.05), sensors, mat_mount)
    add_box("nadir_box_r", (0.01042, -0.41808, 0.72919), (0.05, 0.05, 0.05), sensors, mat_mount)
    for side, y in (("l", 0.41808), ("r", -0.41808)):
        look = look_from_ypr(0.0, 0.9411)
        origin = Vector((0.03542, y, 0.70419))
        add_cyl(
            f"nadir_lens_{side}",
            origin + look * 0.006,
            0.010,
            0.006,
            lenses,
            mat_lens,
            quat=quat_from_to((0, 0, 1), look),
            segments=16,
        )

    # Stereo mast from the deck up to the bar, then the bar and three lenses.
    add_cyl("stereo_mast", (0.115, 0.0, 0.47), 0.012, 0.62, sensors, mat_mount, segments=12)
    add_box("stereo_bar", (0.125, 0.0, 0.78), (0.04, 0.18, 0.045), sensors, mat_mount)
    add_box("oak_body", (0.145, 0.0, 0.785), (0.035, 0.06, 0.04), sensors, mat_mount)
    for name, y, pitch in (
        ("stereo_l", 0.05, 0.3469),
        ("stereo_r", -0.05, 0.3469),
        ("oak", 0.0, 0.0873),
    ):
        look = look_from_ypr(0.0, pitch)
        origin = Vector((0.14, y, 0.78))
        add_cyl(
            f"lens_{name}",
            origin + look * 0.012,
            0.012,
            0.008,
            lenses,
            mat_lens,
            quat=quat_from_to((0, 0, 1), look),
            segments=16,
        )

    # Radar node rotation 0 1 0 1.5708 pitches +X to -Z, so the dish faces down.
    radar = Vector((0.16, 0.0, 0.45))
    down = look_from_ypr(0.0, 1.5708)
    add_box("radar_body", radar + Vector((0.0, 0.0, 0.018)), (0.04, 0.05, 0.028), sensors, mat_mount)
    add_cyl("radar_pedestal", (0.145, 0.0, 0.32), 0.01, 0.32, sensors, mat_mount, segments=12)
    add_cyl(
        "radar_dish",
        radar + down * 0.01,
        0.022,
        0.008,
        lenses,
        mat_lens,
        quat=quat_from_to((0, 0, 1), down),
        segments=20,
    )

    # Rear RGB-D looks along -X. Housing bridges the torso back to the lens.
    rear = Vector((-0.16, 0.0, 0.28))
    rear_look = look_from_ypr(math.pi, 0.0)
    add_box("rear_body", (-0.125, 0.0, 0.28), (0.07, 0.06, 0.05), sensors, mat_mount)
    add_cyl(
        "rear_lens",
        rear + rear_look * 0.008,
        0.014,
        0.008,
        lenses,
        mat_lens,
        quat=quat_from_to((0, 0, 1), rear_look),
        segments=16,
    )

    # Floor camera looks straight down from z = 0.04. The pod hangs under the
    # deck and is not part of the deck collider.
    add_cyl("floor_pod", (0.0, 0.0, 0.075), 0.028, 0.06, sensors, mat_mount, segments=16)
    floor_look = look_from_ypr(0.0, 1.5708)
    add_cyl(
        "floor_lens",
        Vector((0.0, 0.0, 0.04)) + floor_look * 0.004,
        0.012,
        0.004,
        lenses,
        mat_lens,
        quat=quat_from_to((0, 0, 1), floor_look),
        segments=16,
    )

    for name, pos, yaw in SONARS:
        look = look_from_ypr(yaw, 0.0)
        origin = Vector(pos)
        if name in ("us_left", "us_right"):
            sign = 1.0 if pos[1] > 0.0 else -1.0
            add_box(
                f"{name}_rail",
                (0.0, sign * 0.155, 0.125),
                (0.02, 0.09, 0.018),
                sensors,
                mat_mount,
            )
        if name in CORNER_SONARS:
            # The device origin sits inside the tire cylinder. The visible
            # cap is the same x,y, just above the tread.
            cap = Vector((pos[0], pos[1], 0.178))
            add_cyl(f"{name}_cap", cap, 0.016, 0.022, sensors, mat_mount, segments=16)
            add_cyl(
                f"{name}_lens",
                cap + look * 0.012,
                0.008,
                0.004,
                lenses,
                mat_lens,
                quat=quat_from_to((0, 0, 1), look),
                segments=12,
            )
        else:
            back = 0.018
            housing = origin - look * back
            add_box(
                f"{name}_body",
                housing,
                (0.04, 0.045, 0.03) if abs(look.z) < 0.1 else (0.04, 0.04, 0.04),
                sensors,
                mat_mount,
                quat=quat_from_to((1, 0, 0), look),
            )
            add_cyl(
                f"{name}_lens",
                origin + look * 0.004,
                0.010,
                0.005,
                lenses,
                mat_lens,
                quat=quat_from_to((0, 0, 1), look),
                segments=16,
            )

    # Local wheel: Z-up cylinder. Hub is inset so it stays inside the tire AABB.
    tire = add_cyl("tire", (0.0, 0.0, 0.0), WHEEL_R, WHEEL_H, wheel_col, mat_tire, segments=40)
    hub = add_cyl("hub", (0.0, 0.0, 0.0), 0.032, WHEEL_H - 0.008, hub_col, mat_hub, segments=24)
    tire.hide_render = True
    hub.hide_render = True

    roll = Matrix.Rotation(math.pi / 2.0, 4, "X")
    for index, loc in enumerate(HUBS):
        for src in (tire, hub):
            dup = src.copy()
            dup.data = src.data
            dup.hide_render = False
            dup.matrix_world = Matrix.Translation(Vector(loc)) @ roll
            show.objects.link(dup)

    # Render-only upper copies so neck/head/arms/torso are in the picture.
    # The export joins torso into the upper file; these objects are the source.
    return {
        "deck": list(deck.objects),
        "torso": list(torso_col.objects),
        "upper_extra": list(upper.objects),
        "arm_l": list(arm_l.objects),
        "arm_r": list(arm_r.objects),
        "sensors": list(sensors.objects),
        "lenses": list(lenses.objects),
        "wheel": [tire],
        "hub": [hub],
        "ground_mat": mat_ground,
    }


def measure(groups):
    deck_c, deck_s = aabb_of(groups["deck"])
    torso_c, torso_s = aabb_of(groups["torso"])
    arm_l_c, arm_l_s = aabb_of(groups["arm_l"])
    arm_r_c, arm_r_s = aabb_of(groups["arm_r"])
    wheel_pts_r = 0.0
    wheel_z = []
    me = world_mesh(groups["wheel"][0])
    for vert in me.vertices:
        wheel_pts_r = max(wheel_pts_r, math.hypot(vert.co.x, vert.co.y))
        wheel_z.append(vert.co.z)
    bpy.data.meshes.remove(me)
    wheel_h = max(wheel_z) - min(wheel_z)
    hub_r = 0.0
    hub_z = []
    me = world_mesh(groups["hub"][0])
    for vert in me.vertices:
        hub_r = max(hub_r, math.hypot(vert.co.x, vert.co.y))
        hub_z.append(vert.co.z)
    bpy.data.meshes.remove(me)
    hub_h = max(hub_z) - min(hub_z)

    if abs(deck_s.x - 0.32) > 1e-4 or abs(deck_s.y - 0.24) > 1e-4:
        raise SystemExit(f"deck footprint changed: {deck_s}")
    if abs(deck_s.z - 0.058) > 1e-4 or abs(deck_c.z - 0.134) > 1e-4:
        raise SystemExit(f"deck height changed: center {deck_c} size {deck_s}")
    if deck_c.z - deck_s.z / 2.0 < 0.10:
        raise SystemExit("deck collision would sit under 0.10 m")
    if abs(wheel_pts_r - WHEEL_R) > 1e-4 or abs(wheel_h - WHEEL_H) > 1e-4:
        raise SystemExit(f"wheel aabb {wheel_pts_r} {wheel_h}")
    if hub_r > WHEEL_R - 1e-4 or hub_h > WHEEL_H - 1e-4:
        raise SystemExit(f"hub escapes the tire aabb {hub_r} {hub_h}")

    torso_origin = Vector((0.0, 0.0, 0.24))
    torso_local = torso_c - torso_origin
    arm_l_local = arm_l_c - Vector((0.0, 0.11, 0.34))
    arm_r_local = arm_r_c - Vector((0.0, -0.11, 0.34))

    dims = {
        "units": "m",
        "frame": "X forward, Y left, Z up",
        "wheel": {
            "collision": "cylinder",
            "radius_m": round(wheel_pts_r, 6),
            "height_m": round(wheel_h, 6),
            "axis_local": "z",
            "proto_rotation": "1 0 0 1.5708",
            "hub_radius_m": round(hub_r, 6),
            "hub_height_m": round(hub_h, 6),
        },
        "body_boxes": [
            {
                "name": "deck",
                "where": "robot",
                "translation": fmt(deck_c),
                "size": fmt(deck_s),
            },
            {
                "name": "torso",
                "where": "torso_solid",
                "translation": fmt(torso_local),
                "size": fmt(torso_s),
            },
            {
                "name": "left_arm",
                "where": "left_arm_solid",
                "translation": fmt(arm_l_local),
                "size": fmt(arm_l_s),
            },
            {
                "name": "right_arm",
                "where": "right_arm_solid",
                "translation": fmt(arm_r_local),
                "size": fmt(arm_r_s),
            },
        ],
        "excluded_from_collision": [
            "neck",
            "head",
            "nadir booms and boxes",
            "stereo mast and bar",
            "oak housing",
            "radar dish and pedestal",
            "eight sonar housings",
            "rear camera housing",
            "floor camera pod",
            "lens discs",
        ],
        "notes": [
            "Corner sonar device origins (z=0.12) lie inside the tire cylinders. Visible caps use the same x,y at z=0.178.",
            "Radar rotation 0 1 0 1.5708 pitches the +X beam to -Z, so the dish faces down. The Radar node is not moved.",
            "Head, neck, and every sensor mount are visual. Body collision is the deck box, the torso box, and the two arm boxes.",
        ],
    }
    return dims


def export(groups, dims):
    ROOT.mkdir(parents=True, exist_ok=True)
    write_obj(ROOT / "butlerbot_skid_deck.obj", groups["deck"])
    write_obj(
        ROOT / "butlerbot_skid_upper.obj",
        groups["torso"] + groups["upper_extra"] + groups["arm_l"] + groups["arm_r"],
    )
    write_obj(ROOT / "butlerbot_skid_sensors.obj", groups["sensors"])
    write_obj(ROOT / "butlerbot_skid_lenses.obj", groups["lenses"])
    write_obj(ROOT / "butlerbot_skid_wheel.obj", groups["wheel"])
    write_obj(ROOT / "butlerbot_skid_hub.obj", groups["hub"])
    (ROOT / "butlerbot_skid_dimensions.json").write_text(
        json.dumps(dims, indent=2) + "\n", encoding="utf-8"
    )


def render(ground_mat):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 32
    scene.cycles.use_denoising = False
    scene.render.resolution_x = 960
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    try:
        scene.view_settings.view_transform = "AgX"
    except TypeError:
        pass

    world = scene.world or bpy.data.worlds.new("world")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.78, 0.80, 0.82, 1.0)
    bg.inputs["Strength"].default_value = 0.85

    sun_data = bpy.data.lights.new("sun", "SUN")
    sun_data.energy = 3.2
    sun = bpy.data.objects.new("sun", sun_data)
    sun.rotation_euler = (math.radians(48), math.radians(8), math.radians(40))
    scene.collection.objects.link(sun)
    fill_data = bpy.data.lights.new("fill", "AREA")
    fill_data.energy = 250
    fill_data.size = 2.0
    fill = bpy.data.objects.new("fill", fill_data)
    fill.location = (-1.2, 1.4, 1.6)
    fill.rotation_euler = (math.radians(55), 0.0, math.radians(220))
    scene.collection.objects.link(fill)

    add_box("ground", (0.0, 0.0, -0.0008), (3.2, 3.2, 0.001), collection("ground"), ground_mat)

    def camera(name, location, target, ortho=None):
        data = bpy.data.cameras.new(name)
        if ortho is None:
            data.type = "PERSP"
            data.lens = 50
        else:
            data.type = "ORTHO"
            data.ortho_scale = ortho
            data.sensor_fit = "VERTICAL"
        obj = bpy.data.objects.new(name, data)
        obj.location = Vector(location)
        direction = Vector(target) - obj.location
        obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        scene.collection.objects.link(obj)
        return obj

    shots = {
        "skid_front.png": camera("cam_front", (1.85, 0.0, 0.48), (0.0, 0.0, 0.40)),
        "skid_side.png": camera("cam_side", (0.0, -1.85, 0.48), (0.0, 0.0, 0.40)),
        "skid_top.png": camera("cam_top", (0.0, 0.0, 2.4), (0.0, 0.0, 0.0), ortho=1.55),
        "skid_three_quarter.png": camera(
            "cam_quarter", (1.45, -1.35, 1.05), (0.0, 0.0, 0.36)
        ),
    }
    # Top view: nose (+X) toward the top of the frame.
    top = bpy.data.objects["cam_top"]
    top.rotation_euler = (0.0, 0.0, -math.pi / 2.0)

    PREVIEW.mkdir(parents=True, exist_ok=True)
    ART.mkdir(parents=True, exist_ok=True)
    for filename, cam in shots.items():
        scene.camera = cam
        out = ART / filename
        scene.render.filepath = str(out)
        bpy.ops.render.render(write_still=True)
        shutil.copy(out, PREVIEW / filename)
        print(f"rendered {out}")


def main():
    groups = build()
    dims = measure(groups)
    export(groups, dims)
    print(json.dumps(dims, indent=2))
    render(groups["ground_mat"])


if __name__ == "__main__":
    main()
