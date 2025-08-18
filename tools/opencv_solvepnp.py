# test_solvepnp_rerun.py
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import rerun as rr

# 初始化 rerun，并自动弹出 Viewer 窗口
rr.init("pnp_visualizer", spawn=True)

# 定义 3D 特征点
object_points = np.array(
    [
        [-135.0 / 2.0, 55.0 / 2.0, 0.0],
        [-135.0 / 2.0, -55.0 / 2.0, 0.0],
        [135.0 / 2.0, -55.0 / 2.0, 0.0],
        [135.0 / 2.0, 55.0 / 2.0, 0.0],
    ],
    dtype=np.float64,
)

# 两组图像中检测到的 2D 点（单位：像素）
image_points_list = [
    [[197.125, 203.125], [191.25, 231.625], [235.875, 236.375], [241.5, 207.375]],
    [[361.5, 241.5], [366.25, 270.0], [416.5, 268.0], [411.5, 239.5]],
]

# 相机内参矩阵与畸变
camera_matrix = np.array(
    [[1600.0, 0.0, 320.0], [0.0, 1705.7, 192.0], [0.0, 0.0, 1.0]], dtype=np.float64
)
dist_coeffs = np.zeros((5, 1), dtype=np.float64)

# 在世界坐标系中绘制基坐标轴
rr.log("base_link", rr.Transform3D(axis_length=100))

# 遍历每个检测结果
for idx, img_pts in enumerate(image_points_list):
    img_pts = np.array(img_pts, dtype=np.float64)

    # 调用 solvePnP 求解位姿
    success, rvec, tvec = cv2.solvePnP(
        object_points, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE
    )
    if not success:
        print(f"solvePnP failed for detection {idx}")
        continue

    # Rodrigues 转换为旋转矩阵，然后构建四元数 (x, y, z, w)
    R_mat, _ = cv2.Rodrigues(rvec)
    print(f"idx = {idx}")
    print(f"r_matrix = {R_mat}")
    print(f"t_vec = {tvec}")
    quat = R.from_matrix(R_mat).as_quat()  # 返回 (x, y, z, w)

    # 绘制半长方体的尺寸，tvec 为中心位置
    half_sizes = np.array([135.0 / 2.0, 55.0 / 2.0, 10.0], dtype=np.float32)

    # 发送 Box3D 及其转换（位姿）
    rr.log(
        f"armor_{idx}",
        rr.Boxes3D(half_sizes=half_sizes, fill_mode=rr.components.FillMode.Solid),
        rr.Transform3D(
            translation=tvec.flatten().astype(np.float32),
            rotation=rr.Quaternion(xyzw=quat),
            axis_length=100,
        ),
    )

# 数据发送完毕后关闭
rr.disconnect()
