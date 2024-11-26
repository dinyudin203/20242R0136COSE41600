# 시각화에 필요한 라이브러리 불러오기
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# PCD 파일 경로 설정
file_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\01_straight_walk\pcd\pcd_000120.pcd"

# PCD 파일 읽기
original_pcd = o3d.io.read_point_cloud(file_path)

# Voxel Downsampling 수행
voxel_size = 0.2  # 필요에 따라 voxel 크기를 조정하세요.
downsample_pcd = original_pcd.voxel_down_sample(voxel_size=voxel_size)

# Radius Outlier Removal (ROR) 적용
cl, ind = downsample_pcd.remove_radius_outlier(nb_points=6, radius=1.2)
ror_pcd = downsample_pcd.select_by_index(ind)

# RANSAC을 사용하여 평면 추정
plane_model, inliers = ror_pcd.segment_plane(distance_threshold=0.1,
                                             ransac_n=3,
                                             num_iterations=2000)

# 도로에 속하지 않는 포인트 (outliers) 추출
final_point = ror_pcd.select_by_index(inliers, invert=True)

# 포인트 클라우드를 NumPy 배열로 변환
points = np.asarray(final_point.points)

# DBSCAN 클러스터링 적용
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(final_point.cluster_dbscan(eps=0.3, min_points=10, print_progress=True))

# 각 클러스터를 색으로 표시
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")

# 노이즈를 제거하고 각 클러스터에 색상 지정
colors = plt.get_cmap("tab20")(labels / (max_label + 1 if max_label > 0 else 1))
colors[labels < 0] = 0  # 노이즈는 검정색으로 표시
final_point.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 필터링 기준 1. 클러스터 내 최대 최소 포인트 수
min_points_in_cluster = 5   # 클러스터 내 최소 포인트 수
max_points_in_cluster = 40  # 클러스터 내 최대 포인트 수

# 필터링 기준 2. 클러스터 내 최소 최대 Z값
min_z_value = -1.5    # 클러스터 내 최소 Z값
max_z_value = 2.5   # 클러스터 내 최대 Z값

# 필터링 기준 3. 클러스터 내 최소 최대 Z값 차이
min_height = 0.5   # Z값 차이의 최소값
max_height = 2.0   # Z값 차이의 최대값

max_distance = 30.0  # 원점으로부터의 최대 거리

# 1번, 2번, 3번 조건을 모두 만족하는 클러스터 필터링 및 바운딩 박스 생성
bboxes_1234 = []
for i in range(max_label + 1):
    cluster_indices = np.where(labels == i)[0]
    if min_points_in_cluster <= len(cluster_indices) <= max_points_in_cluster:
        cluster_pcd = final_point.select_by_index(cluster_indices)
        points = np.asarray(cluster_pcd.points)
        z_values = points[:, 2]  # Z값 추출
        z_min = z_values.min()
        z_max = z_values.max()
        if min_z_value <= z_min and z_max <= max_z_value:
            height_diff = z_max - z_min
            if min_height <= height_diff <= max_height:
                distances = np.linalg.norm(points, axis=1)
                if distances.max() <= max_distance:
                    bbox = cluster_pcd.get_axis_aligned_bounding_box()
                    bbox.color = (1, 0, 0) 
                    bboxes_1234.append(bbox)

# 2D 투영 및 시각화 함수
def visualize_2d_projection_with_clusters(pcd, labels, bounding_boxes, point_size=1.0):
    # 점 구름 데이터를 NumPy 배열로 변환
    points = np.asarray(pcd.points)
    cluster_ids = np.array(labels)
    
    # XY 평면에 투영
    points_xy = points[:, :2]
    
    # 클러스터 중심 계산
    cluster_centroids = []
    for i in range(max(cluster_ids) + 1):
        cluster_points = points[cluster_ids == i]
        if len(cluster_points) > 0:
            centroid = cluster_points.mean(axis=0)
            cluster_centroids.append(centroid[:2])  # XY 좌표
    
    # 시각화
    plt.figure(figsize=(10, 8))
    scatter = plt.scatter(points_xy[:, 0], points_xy[:, 1], c=cluster_ids, cmap="tab20", s=point_size)
    plt.colorbar(scatter, label="Cluster ID")
    
    # 클러스터 중심 표시
    for i, centroid in enumerate(cluster_centroids):
        plt.text(centroid[0], centroid[1], f"{i}", color="red", fontsize=5, ha="center", va="center")

    plt.title("2D Projection of Point Cloud (XY plane)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.show()

# 클러스터 필터링 후 2D 시각화
visualize_2d_projection_with_clusters(final_point, labels, bboxes_1234, point_size=10)