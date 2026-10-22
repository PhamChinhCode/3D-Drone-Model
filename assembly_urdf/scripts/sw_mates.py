"""List mates touching the camera/servo parts of Assem1.SLDASM (type, components, entity params)."""
import sys
import win32com.client, pythoncom

ASM = sys.argv[1]
KEYS = ("GiaCam3A", "Servo", "Camera Module", "GiaCam3-")
TYPES = {0: "coincident", 1: "concentric", 2: "perpendicular", 3: "parallel", 4: "tangent",
         5: "distance", 6: "angle", 7: "unknown", 8: "symmetric", 9: "camfollower", 10: "gear",
         11: "width", 12: "lock", 13: "screw", 14: "linearcoupler", 15: "universal",
         16: "coordinate", 17: "slot", 18: "hinge", 19: "slider", 20: "profilecenter",
         21: "magnetic"}
sw = win32com.client.Dispatch("SldWorks.Application")
sw.Visible = False
e = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
w = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
doc = sw.OpenDoc6(ASM, 2, 1 | 2, "", e, w)
doc.ResolveAllLightWeightComponents(False)

feat = doc.FirstFeature
while feat:
    if feat.GetTypeName2 == "MateGroup":
        sub = feat.GetFirstSubFeature
        while sub:
            mate = sub.GetSpecificFeature2
            try:
                n = mate.GetMateEntityCount
            except Exception:
                n = 0
            comps = []
            for i in range(n):
                me = mate.MateEntity(i)
                c = me.ReferenceComponent
                comps.append((c.Name2 if c else "?", me.ReferenceType2, [round(v, 5) for v in me.EntityParams]))
            if any(k in c[0] for c in comps for k in KEYS):
                print(f"{sub.Name} | {TYPES.get(mate.Type, mate.Type)} | suppressed={sub.IsSuppressed}")
                for c in comps:
                    print("    ", c)
            sub = sub.GetNextSubFeature
    feat = feat.GetNextFeature
sw.CloseDoc(doc.GetTitle)
