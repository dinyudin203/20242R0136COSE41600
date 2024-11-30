import cv2
import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt

# 설정값
video_output_path = "output_video.mp4"
voxel_size = 0.2
min_points_in_cluster = 5
max_points_in_cluster = 40
min_z_value = -1.5
max_z_value = 2.5
min_height = 0.5
max_height = 2.0
max_distance = 30.0
movement_threshold = 0.1
# 파일 경로 설정
folder_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\01_straight_walk\pcd"
file_list = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith(".pcd")])

# 클러스터 중심점 계산 함수
def get_cluster_centers(pcd, labels, max_label):
    centers = []
    for i in range(max_label + 1):
        cluster_indices = np.where(labels == i)[0]
        if len(cluster_indices) >= min_points_in_cluster:
            cluster_pcd = pcd.select_by_index(cluster_indices)
            points = np.asarray(cluster_pcd.points)
            centers.append(np.mean(points, axis=0))  # 클러스터 중심점 계산
    return np.array(centers)

# Open3D 시각화 및 이미지 저장
image_dir = "frames"
os.makedirs(image_dir, exist_ok=True)
prev_centers = None

for file_idx, file_path in enumerate(file_list):
    # PCD 파일 읽기 및 Downsampling
    pcd = o3d.io.read_point_cloud(file_path)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # ROR 적용
    cl, ind = pcd.remove_radius_outlier(nb_points=6, radius=1.2)
    pcd = pcd.select_by_index(ind)
    
    # 평면 제거
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=2000)
    pcd = pcd.select_by_index(inliers, invert=True)
    
    # DBSCAN 클러스터링
    labels = np.array(pcd.cluster_dbscan(eps=0.3, min_points=10, print_progress=False))
    max_label = labels.max()
    
    # 모든 포인트를 검은색으로 초기화
    black_color = np.zeros((len(pcd.points), 3))
    pcd.colors = o3d.utility.Vector3dVector(black_color)

    # 현재 클러스터 중심 계산
    centers = get_cluster_centers(pcd, labels, max_label)
    
    current_bboxes = []
    if file_idx == 0:
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            if min_points_in_cluster <= len(cluster_indices) <= max_points_in_cluster:
                cluster_pcd = pcd.select_by_index(cluster_indices)
                points = np.asarray(cluster_pcd.points)
                z_values = points[:, 2]
                z_min, z_max = z_values.min(), z_values.max()
                if min_z_value <= z_min and z_max <= max_z_value:
                    height_diff = z_max - z_min
                    if min_height <= height_diff <= max_height:
                        if np.linalg.norm(points, axis=1).max() <= max_distance:
                            bbox = cluster_pcd.get_axis_aligned_bounding_box()
                            bbox.color = (1, 0, 0)
                            current_bboxes.append(bbox)
                            for idx in cluster_indices:
                                pcd.colors[idx] = [1.0, 0.0, 0.0]
    else:
        if prev_centers is not None:
            for i, center in enumerate(centers):
                if prev_centers.shape == 0 or centers.shape == 0:
                    continue
                distances = np.linalg.norm(prev_centers[:len(centers)] - center, axis=1)
                if distances.min() < movement_threshold:
                    cluster_indices = np.where(labels == i)[0]
                    if len(cluster_indices) < min_points_in_cluster:
                        continue
                    cluster_pcd = pcd.select_by_index(cluster_indices)
                    points = np.asarray(cluster_pcd.points)
                    z_values = points[:, 2]
                    z_min, z_max = z_values.min(), z_values.max()
                    if min_z_value <= z_min and z_max <= max_z_value:
                        height_diff = z_max - z_min
                        if min_height <= height_diff <= max_height:
                            if np.linalg.norm(points, axis=1).max() <= max_distance:
                                bbox = cluster_pcd.get_axis_aligned_bounding_box()
                                bbox.color = (0, 1, 0)
                                current_bboxes.append(bbox)
                                for idx in cluster_indices:
                                    pcd.colors[idx] = [0.0, 1.0, 0.0]
    
    prev_centers = centers
    
    # Open3D 시각화
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)  # 창을 숨김
    vis.add_geometry(pcd)
    for bbox in current_bboxes:
        vis.add_geometry(bbox)
    vis.poll_events()
    vis.update_renderer()
    
    # 이미지 저장
    image_path = os.path.join(image_dir, f"frame_{file_idx:04d}.png")
    vis.capture_screen_image(image_path)
    vis.destroy_window()

# OpenCV로 동영상 생성
frame_paths = sorted([os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith(".png")])
frame = cv2.imread(frame_paths[0])
height, width, _ = frame.shape
video_writer = cv2.VideoWriter(video_output_path, cv2.VideoWriter_fourcc(*"mp4v"), 10, (width, height))

for frame_path in frame_paths:
    frame = cv2.imread(frame_path)
    video_writer.write(frame)

video_writer.release()
print(f"Video saved to {video_output_path}")
