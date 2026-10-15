# 赛题人脸底库

题目提供的 10 人正脸照。子目录名即识别身份（`1` … `10`），与题目文件名一致。

```
data/faces/
  1/1.jpg
  2/2.jpg
  ...
  10/10.jpg
```

离线批量注册（在 `vision/arcface-lite/` 下）：

```bash
.venv/bin/python engine.py register ../../data/faces face_db.npz --det_size 640
```

`face_db.npz` 是本机特征库，不入库。换 `buffalo_sc` / `buffalo_l` 后须重新注册。
