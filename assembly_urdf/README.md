# drone_v2 URDF

URDF của `Assem1.SLDASM` (khung Version2.0): `base_link` + 4 cánh quạt (joint `continuous`).

```
assembly_urdf/
  drone_v2.urdf        mô tả robot (sổ ghi khung/giả định ở đầu file)
  meshes/              STL theo từng link, đơn vị mét, đã ở hệ trục của link
  scripts/sw_tess.py   đọc Assem1.SLDASM qua COM SolidWorks -> leaves.pkl
  scripts/build_meshes.py  leaves.pkl -> meshes/*.stl + in số quán tính
```

## Hệ trục

- `base_link`: gốc ở tâm 4 trục motor, cao bằng mặt bo FC; X = mũi, Y = trái, Z = lên (REP-103).
- Mũi được **giả định** là hướng về phía camera (đường chéo +X+Z của assembly). Kiểm lại với mũi tên trên FC.
- Cánh quạt: trước-trái và sau-phải là CW (`axis 0 0 -1`), trước-phải và sau-trái là CCW (`axis 0 0 1`).

## Khối lượng

Lấy từ mass properties của SolidWorks, tổng 0,768 kg. Gần như mọi chi tiết đang dùng mật độ mặc định 1000 kg/m³,
nên khối lượng **chỉ là ước tính** (motor CAD 9,7 g so với ~50 g thật). Cân drone rồi sửa trước khi mô phỏng động lực học.

## Tạo lại sau khi sửa CAD

```bash
python scripts/sw_tess.py "<đường dẫn>/Assem1.SLDASM" leaves.pkl
python scripts/build_meshes.py leaves.pkl meshes
```

Sau đó chép các số quán tính và vị trí joint mà script in ra vào `drone_v2.urdf`, rồi chạy `cadgen urdf validate drone_v2.urdf`.
