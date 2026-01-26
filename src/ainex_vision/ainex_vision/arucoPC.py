import cv2
from cv2 import aruco

# 1. 选择字典（和 ROS 节点保持一致）
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 2. 选择 ID
marker_id = 0       # 0~49 都可以
marker_size = 400   # 像素大小

# 3. 生成 ArUco 标记
marker_img = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# 4. 保存图片
cv2.imwrite(f"aruco_id{marker_id}.png", marker_img)
print(f"✅ 已生成 ArUco 标记：aruco_id{marker_id}.png")
