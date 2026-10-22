"""Tessellate every visible leaf part of the assembly + its mass props, in part frame.
Writes leaves.pkl: list of dict(name, path, xform[13], tris(N,3,3) m, mass, com, moi[9])."""
import pickle, sys
import numpy as np
import win32com.client, pythoncom

ASM, OUT = sys.argv[1], sys.argv[2]
sw = win32com.client.Dispatch("SldWorks.Application")
sw.Visible = False
e = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
w = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
doc = sw.OpenDoc6(ASM, 2, 1 | 2, "", e, w)
doc.ResolveAllLightWeightComponents(False)
root = doc.ConfigurationManager.ActiveConfiguration.GetRootComponent3(True)

leaves = []

def part_data(comp):
    m = comp.GetModelDoc2
    tris = []
    bodies = comp.GetBodies2(0)  # solid bodies, part space
    for b in bodies or []:
        for f in b.GetFaces() or []:
            t = f.GetTessTriangles(True)
            if t:
                tris.append(np.array(t, float).reshape(-1, 3, 3))
    d = {"tris": np.concatenate(tris) if tris else np.zeros((0, 3, 3))}
    mp = m.Extension.CreateMassProperty
    d["mass"] = mp.Mass
    d["com"] = list(mp.CenterOfMass)
    d["moi"] = list(mp.GetMomentOfInertia(0))
    return d

def walk(comp, top):
    for ch in comp.GetChildren or []:
        if ch.IsSuppressed or ch.IsHidden(True):
            continue
        kids = ch.GetChildren
        if kids:
            walk(ch, top or ch.Name2)
            continue
        if ch.GetModelDoc2 is None or ch.GetModelDoc2.GetType != 1:
            continue
        d = part_data(ch)
        d.update(name=ch.Name2, top=top or ch.Name2, path=ch.GetPathName,
                 xform=list(ch.Transform2.ArrayData))
        leaves.append(d)
        print(d["top"], "|", d["name"], len(d["tris"]), "%.4f" % d["mass"], flush=True)

walk(root, None)
mp = doc.Extension.CreateMassProperty
pickle.dump({"leaves": leaves, "asm_mass": mp.Mass, "asm_com": list(mp.CenterOfMass),
             "asm_moi": list(mp.GetMomentOfInertia(0))}, open(OUT, "wb"))
sw.CloseDoc(doc.GetTitle)
