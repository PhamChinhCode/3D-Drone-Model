"""Turn the SolidWorks tessellation dump (sw_tess.py -> leaves.pkl) into URDF link meshes
and print the inertial numbers used in drone_v2.urdf.

Usage:  python build_meshes.py leaves.pkl ../meshes

Frames (see the ledger in drone_v2.urdf):
  assembly (SolidWorks Assem1): +Y up, motors on the +/-X and +/-Z assembly axes.
  base_link: origin = centre of the 4 motor axes, at the FC board plane (fc-1 origin);
             x = nose (diagonal towards camera, +X+Z assembly), y = left, z = up.
  prop_N:    origin = propeller hub (prop part origin), axes parallel to base_link.
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

base = [l for l in leaves if l["top"] not in PROPS]
save("base_link", np.concatenate([leaf_tris_base(l) for l in base if VISUAL_SKIP not in l["name"]]))
report("base_link", *combine([leaf_inertia_base(l) for l in base]))

for top, link in PROPS.items():
    l = by_top(top)[0]
    hub = to_base(hubs[top])
    print(f"joint {link}: origin xyz={hub.round(5)}")
    save(link, leaf_tris_base(l) - hub)
    report(link, *combine([leaf_inertia_base(l)], hub))

# cross-check against SolidWorks whole-assembly mass properties (assembly axes, about COM)
m, c, I = combine([leaf_inertia_base(l) for l in leaves])
I_asm = RB.T @ I @ RB
I_sw = np.array(d["asm_moi"]).reshape(3, 3)
I_sw = 2 * np.diag(np.diag(I_sw)) - I_sw
print("total mass", round(m, 5), "SW", round(d["asm_mass"], 5))
print("max |I - I_SW| =", np.abs(I_asm - I_sw).max())
