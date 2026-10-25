"""Turn the SolidWorks tessellation dump (sw_tess.py -> leaves.pkl) into URDF link meshes
and print the inertial numbers used in drone_v2.urdf.

Usage:  python build_meshes.py leaves.pkl ../meshes

Frames (see the ledger in drone_v2.urdf):
  assembly (SolidWorks Assem1): +Y up, motors on the +/-X and +/-Z assembly axes.
  base_link: origin = centre of the 4 motor axes, at the FC board plane (fc-1 origin);
             x = nose (diagonal towards camera, +X+Z assembly), y = left, z = up.
  prop_N:    origin = propeller hub (prop part origin), axes parallel to base_link.
  camera_tilt_link: origin on the servo output axis; at joint 0 the camera looks straight
             down (axes parallel to base_link), geometry is rotated back from its CAD pose.
"""
import pickle, sys
import numpy as np
import trimesh
import fast_simplification

src, out_dir = sys.argv[1], sys.argv[2]
d = pickle.load(open(src, "rb"))
leaves = d["leaves"]

PROPS = {  # assembly component -> URDF link
    "PROPELLER - 8x4.5 CW-1": "prop_front_left",
    "PROPELLER - 8x4.5 CCW-2": "prop_front_right",
    "PROPELLER - 8x4.5 CW-2": "prop_rear_right",
    "PROPELLER - 8x4.5 CCW-1": "prop_rear_left",
}
MAX_TRIS_PER_LEAF = 4000  # decimate heavy vendor STEP parts (Pi heatsink, nuts)
VISUAL_SKIP = "Stator_Coil"  # 48 coils hidden inside the motor bells, ~94k tris each and
                             # don't decimate; dropped from the visual mesh, kept for inertia
# GiaCam3A turns on the servo output shaft; the camera module is mated to it.
CAMERA = ("GiaCam3A-1", "Camera Module Assem.STEP-1")
# Assembly-space mate data read with sw_mates.py (Assem1.SLDASM, 2026-10-22, lens re-read 10-25):
SERVO_AXIS_POINT_ASM = np.array([0.3502, 0.01644, 0.20609])  # Concentric91, servo shaft
SERVO_AXIS_DIR_ASM = np.array([-0.70711, 0.0, 0.70711])
LENS_DIR_ASM = np.array([0.69992, 0.14223, 0.69992])  # Concentric92, sign: towards Lenz part

# Gripper (2026-10-25): MG90 double horn drives two crossed CauNoi rods, each rod drives one GapHang
# jaw on its pin in GiaPin-1. A closed four-bar per side; URDF keeps the tree and cuts the loop at
# the rod -> jaw ear pin. All pins are parallel to base_link X. Assembly-space mate points
# (sw_mates.py, Assem1.SLDASM 10-25): horn centre = midpoint of the two horn holes.
GRIP_HORN = ("MG90servo_assembly-1/MG90servo_gear-1", "MG90servo_assembly-1/MG90servo_shaft-1")
GRIP_HOLE_L_ASM = np.array([0.28743, -0.01607, 0.13097])   # Concentric122, horn <-> CauNoi-2
GRIP_HOLE_R_ASM = np.array([0.30054, -0.01592, 0.11786])   # Concentric121, horn <-> CauNoi-3
GRIP_EAR_L_ASM = np.array([0.3184, -0.01572, 0.1])         # Concentric117, CauNoi-2 <-> GapHang-1
GRIP_EAR_R_ASM = np.array([0.26957, -0.01572, 0.14883])    # Concentric118, CauNoi-3 <-> GapHang-2
GRIP_PIVOT_L_ASM = np.array([0.31962, 0.00612, 0.1055])    # Concentric101, GiaPin-1 <-> GapHang-1
GRIP_PIVOT_R_ASM = np.array([0.27507, 0.00612, 0.15005])   # Concentric102, GiaPin-1 <-> GapHang-2
GRIP_LINKS = {  # assembly component -> (URDF link, joint origin in assembly space)
    "GapHang-1": ("gripper_jaw_left", GRIP_PIVOT_L_ASM),     # +Y side of base_link
    "GapHang-2": ("gripper_jaw_right", GRIP_PIVOT_R_ASM),
    "CauNoi-2": ("gripper_rod_left", GRIP_HOLE_L_ASM),       # drives the left jaw
    "CauNoi-3": ("gripper_rod_right", GRIP_HOLE_R_ASM),
}
GRIP_OPEN_DEG = -180.0  # full MG90 travel: second toggle, jaws fully open (56 deg), self locking

def xf(l):
    x = np.array(l["xform"])
    return x[:9].reshape(3, 3), x[9:12]  # SolidWorks row-vector convention: p_asm = p @ R + t

def by_top(name):
    return [l for l in leaves if l["top"] == name]

# base_link frame in assembly coordinates
fc_y = xf(by_top("fc-1")[0])[1][1]
ORIGIN = np.array([0.0, fc_y, 0.0])
hubs = {l["top"]: xf(l)[1] for l in leaves if l["top"] in PROPS}  # props sit on the motor axes
ORIGIN[0] = np.mean([h[0] for h in hubs.values()])
ORIGIN[2] = np.mean([h[2] for h in hubs.values()])
s = 1 / np.sqrt(2)
RB = np.array([[s, 0, s],     # x_b (nose) in assembly coords
               [s, 0, -s],    # y_b (left)
               [0, 1, 0]])    # z_b (up)

def to_base(p_asm):
    return (p_asm - ORIGIN) @ RB.T

def leaf_tris_base(l):
    R, t = xf(l)
    tri = l["tris"]
    if len(tri) > MAX_TRIS_PER_LEAF:
        m = trimesh.Trimesh(*trimesh.util.append_faces([tri.reshape(-1, 3)],
                                                       [np.arange(len(tri) * 3).reshape(-1, 3)]))
        m.merge_vertices()
        v, f = fast_simplification.simplify(m.vertices, m.faces,
                                            target_reduction=1 - MAX_TRIS_PER_LEAF / len(m.faces))
        tri = v[f]
    return to_base(tri.reshape(-1, 3) @ R + t).reshape(-1, 3, 3)

def leaf_inertia_base(l):
    """mass, COM (base frame), inertia about own COM in base axes."""
    R, t = xf(l)
    rc = RB @ R.T                       # part axes -> base axes (column convention)
    I = np.array(l["moi"]).reshape(3, 3)
    I = 2 * np.diag(np.diag(I)) - I    # SolidWorks gives products Pxy=+∫xy dm; tensor is -Pxy
    return l["mass"], to_base(np.array(l["com"]) @ R + t), rc @ I @ rc.T

def combine(parts, frame_origin=np.zeros(3)):
    m = sum(p[0] for p in parts)
    c = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        r = ci - c
        I += Ii + mi * (r @ r * np.eye(3) - np.outer(r, r))
    return m, c - frame_origin, I

def save(name, tris):
    mesh = trimesh.Trimesh(vertices=tris.reshape(-1, 3), faces=np.arange(len(tris) * 3).reshape(-1, 3))
    mesh.merge_vertices()
    mesh.export(f"{out_dir}/{name}.stl")
    lo, hi = mesh.bounds
    print(f"{name}.stl  tris={len(mesh.faces)}  bbox_min={lo.round(4)} bbox_max={hi.round(4)}")

def report(name, m, c, I):
    print(f"[{name}] mass={m:.5f} com={c.round(5)}\n  ixx={I[0,0]:.4e} iyy={I[1,1]:.4e} izz={I[2,2]:.4e}"
          f" ixy={I[0,1]:.3e} ixz={I[0,2]:.3e} iyz={I[1,2]:.3e}")

base = [l for l in leaves if l["top"] not in PROPS and l["top"] not in CAMERA
        and l["top"] not in GRIP_LINKS and l["name"] not in GRIP_HORN]
save("base_link", np.concatenate([leaf_tris_base(l) for l in base if VISUAL_SKIP not in l["name"]]))
report("base_link", *combine([leaf_inertia_base(l) for l in base]))

for top, link in PROPS.items():
    l = by_top(top)[0]
    hub = to_base(hubs[top])
    print(f"joint {link}: origin xyz={hub.round(5)}")
    save(link, leaf_tris_base(l) - hub)
    report(link, *combine([leaf_inertia_base(l)], hub))

# camera tilt: joint axis 0 -1 0 so positive angle turns the view from straight down towards
# the nose, same as camera_tilt.py (tilt_deg + towards nose, 0 = straight down)
axis = RB @ SERVO_AXIS_DIR_ASM
assert np.allclose(np.abs(axis), [0, 1, 0], atol=1e-4), axis
pivot = to_base(SERVO_AXIS_POINT_ASM)
lens = RB @ LENS_DIR_ASM
q_cad = np.arctan2(lens[0], -lens[2])
c, s_ = np.cos(q_cad), np.sin(q_cad)
RQ = np.array([[c, 0, -s_], [0, 1, 0], [s_, 0, c]])  # rotation about -Y by q_cad, base <- child
print(f"joint camera_tilt_joint: origin xyz={pivot.round(5)}  CAD pose tilt={np.degrees(q_cad):.1f} deg")
cam = [l for l in leaves if l["top"] in CAMERA]
save("camera_tilt_link", ((np.concatenate([leaf_tris_base(l) for l in cam]) - pivot) @ RQ))
m, com, I = combine([leaf_inertia_base(l) for l in cam])
report("camera_tilt_link", m, RQ.T @ (com - pivot), RQ.T @ I @ RQ)

# gripper: every link frame is parallel to base_link and sits on its pin; joint 0 = CAD pose
horn_c = to_base((GRIP_HOLE_L_ASM + GRIP_HOLE_R_ASM) / 2)

def save_rigid(link, parts, origin):
    save(link, np.concatenate([leaf_tris_base(l) for l in parts]) - origin)
    report(link, *combine([leaf_inertia_base(l) for l in parts], origin))

print(f"joint gripper_servo_joint: origin xyz={horn_c.round(5)}")
save_rigid("gripper_servo_link", [l for l in leaves if l["name"] in GRIP_HORN], horn_c)
for top, (link, o) in GRIP_LINKS.items():
    o = to_base(o)
    rel = o - horn_c if "rod" in link else o
    print(f"joint {link}_joint: origin xyz={rel.round(5)} ({'servo link' if 'rod' in link else 'base'})")
    save_rigid(link, by_top(top), o)

# four-bar per side in the base Y-Z plane, rotation about +X; mimic = line through the exact
# closed (0) and open (GRIP_OPEN_DEG) poses, so both end states display exactly
yz = lambda p: to_base(p)[1:]
rot = lambda v, a: np.array([np.cos(a) * v[0] - np.sin(a) * v[1], np.sin(a) * v[0] + np.cos(a) * v[1]])
ang = lambda v: np.arctan2(v[1], v[0])
H = yz((GRIP_HOLE_L_ASM + GRIP_HOLE_R_ASM) / 2)

def four_bar(hole, ear, pivot, th):
    """(jaw angle, rod angle relative to the horn) at servo angle th, branch of the CAD pose."""
    h0, e0, p = yz(hole), yz(ear), yz(pivot)
    L, R = np.linalg.norm(e0 - h0), np.linalg.norm(e0 - p)
    h = H + rot(h0 - H, th)
    dv = p - h
    dist = np.linalg.norm(dv)
    a = (L * L - R * R + dist * dist) / (2 * dist)
    u = dv / dist
    n = np.array([-u[1], u[0]]) * np.sqrt(L * L - a * a)
    side = np.sign(np.cross(np.r_[p - h0, 0], np.r_[e0 - h0, 0])[2])
    e = next(c for c in (h + a * u + n, h + a * u - n)
             if np.sign(np.cross(np.r_[dv, 0], np.r_[c - h, 0])[2]) == side)
    # rods turn past 180 deg towards the full opening: keep their angle in [-90, 270) deg
    rod = (ang(e - h) - ang(e0 - h0) - th + np.pi / 2) % (2 * np.pi) - np.pi / 2
    return ang(e - p) - ang(e0 - p), rod

ths = np.radians(np.linspace(GRIP_OPEN_DEG, 0, 121))
for name, hole, ear, piv in (("left", GRIP_HOLE_L_ASM, GRIP_EAR_L_ASM, GRIP_PIVOT_L_ASM),
                             ("right", GRIP_HOLE_R_ASM, GRIP_EAR_R_ASM, GRIP_PIVOT_R_ASM)):
    jaw, rod = np.array([four_bar(hole, ear, piv, t) for t in ths]).T
    for what, y in (("jaw", jaw), ("rod", rod)):
        k = y[0] / ths[0]
        err = np.degrees(np.abs(k * ths - y).max())
        print(f"mimic gripper_{what}_{name}_joint: multiplier={k:.4f}  max err {err:.1f} deg"
              f"  (exact at open {y[0]:.5f} rad)")
    print(f"  ear pin distances L={np.linalg.norm(yz(ear) - yz(hole)):.5f}"
          f" R={np.linalg.norm(yz(ear) - yz(piv)):.5f}")

# cross-check against SolidWorks whole-assembly mass properties (assembly axes, about COM)
m, c, I = combine([leaf_inertia_base(l) for l in leaves])
I_asm = RB.T @ I @ RB
I_sw = np.array(d["asm_moi"]).reshape(3, 3)
I_sw = 2 * np.diag(np.diag(I_sw)) - I_sw
print("total mass", round(m, 5), "SW", round(d["asm_mass"], 5))
print("max |I - I_SW| =", np.abs(I_asm - I_sw).max())
