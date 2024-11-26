import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def calculate_optical_flow(pcd1, pcd2):
    # 두 PCD에서 포인트 가져오기
    points1 = np.asarray(pcd1.points)
    points2 = np.asarray(pcd2.points)

    # K-D 트리로 최근접 이웃 검색
    pcd_tree = o3d.geometry.KDTreeFlann(pcd2)
    flow_vectors = []
    valid_points = []

    for point in points1:
        [k, idx, _] = pcd_tree.search_knn_vector_3d(point, 1)  # 가장 가까운 이웃 검색
        if k > 0:
            matched_point = points2[idx[0]]
            flow_vectors.append(matched_point - point)
            valid_points.append(point)

    return np.array(valid_points), np.array(flow_vectors)

def visualize_optical_flow(points, flow_vectors, scale=1.0):
    # 3D Optical Flow 시각화
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 원래 포인트 시각화
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', label='Points', s=1)

    # Optical Flow 시각화 (화살표)
    for i in range(len(points)):
        start = points[i]
        end = points[i] + flow_vectors[i] * scale
        ax.quiver(start[0], start[1], start[2],
                  flow_vectors[i, 0], flow_vectors[i, 1], flow_vectors[i, 2],
                  color='red', length=scale, normalize=True)

    ax.set_title("Optical Flow Visualization (Point Cloud)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()

# Example Usage
pcd1_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd\pcd_000060.pcd"
pcd2_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd\pcd_000061.pcd"

pcd1 = o3d.io.read_point_cloud(pcd1_path)
pcd2 = o3d.io.read_point_cloud(pcd2_path)

# Optical Flow 계산
valid_points, flow_vectors = calculate_optical_flow(pcd1, pcd2)

# Optical Flow 시각화
visualize_optical_flow(valid_points, flow_vectors, scale=0.1)
