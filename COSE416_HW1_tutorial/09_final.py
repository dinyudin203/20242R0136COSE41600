import open3d as o3d
import os
import numpy as np
import time
from scipy.spatial import KDTree
import cv2


folder_paths = [
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/01_straight_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/02_straight_duck_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/03_straight_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/04_zigzag_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/05_straight_duck_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/06_straight_crawl/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/07_straight_walk/pcd',
]
output_folder = 'E:/Downloads/COSE416_HW1_tutorial/output'
output_file = 'E:/Downloads/COSE416_HW1_tutorial/output/02_straight_walk.mp4'
os.makedirs(output_folder, exist_ok=True)

frame_width = 800
frame_height = 600
fps = 10


# Voxel Downsampling 설정
voxel_size = 0.15
point_size = 0.5
frame_delay = 0.3

# 첫 번째 폴더의 PCD 파일들
folder = folder_paths[1]
file_paths = sorted([os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".pcd")])

min_z_value = -1.5
max_z_value = 2.5
min_height = 0.5
max_height = 1.5

max_distance = 30.0

min_width = 0.08
max_width = 0.5

min_volume = 0.1
max_volume = 0.4

min_movement_distance = 0.02
max_movement_distance = 0.4

max_points_in_cluster = 40
# Visualizer 생성 (고정된 창 크기)
vis = o3d.visualization.Visualizer()
vis.create_window(width=800, height=600)
vis.get_render_option().point_size = point_size

# 카메라 컨트롤 객체
view_control = vis.get_view_control()

def calculate_cluster_center(cluster_pcd):
    points = np.asarray(cluster_pcd.points)
    center = np.mean(points, axis=0)
    return center

def calculate_movement(previous_center, current_center):
    distance = np.linalg.norm(np.array(previous_center) - np.array(current_center))
    return distance

def match_clusters(previous_centers, current_center):
    if not previous_centers:
        return None, None
    tree = KDTree(list(previous_centers.values()))
    distance, index = tree.query(current_center)
    if distance > 30:
        return None, None
    return list(previous_centers.keys())[index], distance

def downsample(pcd):
    downsample_pcd = pcd.voxel_down_sample(voxel_size=0.08)
    cl, ind = downsample_pcd.remove_radius_outlier(nb_points=2, radius=0.4)
    ror_pcd = downsample_pcd.select_by_index(ind)
    plane_model, inliers = downsample_pcd.segment_plane(distance_threshold=0.1, ransac_n=5, num_iterations=2000)
    final_point = downsample_pcd.select_by_index(inliers, invert=True)

    return final_point


# 첫 번째 PCD 파일 로드 및 초기화
print(f"Loading: {file_paths[0]}")
pcd = o3d.io.read_point_cloud(file_paths[0])
downsample_pcd = downsample(pcd)
bboxes = []
# 색상 설정
downsample_pcd.paint_uniform_color([0, 0, 0])  # 다운샘플된 PCD를 파란색으로 설정

# Visualizer에 추가
vis.add_geometry(downsample_pcd)

# 중심점 저장용 딕셔너리
previous_centers = {}


with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(downsample_pcd.cluster_dbscan(eps=0.8, min_points=8, print_progress=True))


for i in range(labels.max() + 1):
    cluster_indices = np.where(labels == i)[0]
    if len(cluster_indices) <= max_points_in_cluster:
        cluster_pcd = downsample_pcd.select_by_index(cluster_indices)
        points = np.asarray(cluster_pcd.points)
        z_values = points[:, 2]
        z_min = z_values.min()
        z_max = z_values.max()
        height_diff = z_max - z_min
        if min_height <= height_diff <= max_height:
            x_width = points[:, 0].max() - points[:, 0].min()
            y_width = points[:, 1].max() - points[:, 1].min()
            if min_width <= x_width <= max_width and min_width <= y_width <= max_width:
                center = calculate_cluster_center(cluster_pcd)
                try:
                    # Bounding box 생성
                    bbox = cluster_pcd.get_oriented_bounding_box()
                    bbox.color = (1, 0, 0)
                    bboxes.append(bbox)
                    vis.add_geometry(bbox)
                except RuntimeError as e:
                    print(f"Bounding box creation failed for cluster {i}: {e}")
                previous_centers[i] = center

# 사용자가 카메라를 조정하도록 대기
print("Adjust the camera view. Close the window to continue.")
while vis.poll_events():
    vis.update_renderer()

# 카메라 뷰 저장
camera_params = view_control.convert_to_pinhole_camera_parameters()

prev_pcd = None

video_writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))
# PCD 파일 순차 처리
frame_start = time.time()
frame = 0
for file_path in file_paths[1:]:
    print(f"Processing: {file_path}")
    
    # 새로운 PCD 데이터 로드
    new_pcd = o3d.io.read_point_cloud(file_path)
    new_downsampled = downsample(new_pcd)
    new_downsampled.paint_uniform_color([0, 0, 0])
    downsample_pcd.points = new_downsampled.points
    downsample_pcd.colors = new_downsampled.colors

    # 클러스터링
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(downsample_pcd.cluster_dbscan(eps=1.2, min_points=4, print_progress=True))
    
    for bbox in bboxes:
        vis.remove_geometry(bbox)
    bboxes = []
    temp_center = {}

    for i in range(labels.max() + 1):
        cluster_indices = np.where(labels == i)[0]
        if len(cluster_indices) <= max_points_in_cluster:
            cluster_pcd = downsample_pcd.select_by_index(cluster_indices)
            points = np.asarray(cluster_pcd.points)
            z_values = points[:, 2]
            z_min = z_values.min()
            z_max = z_values.max()
            height_diff = z_max - z_min
            if min_height <= height_diff <= max_height:
                width = max(points[:, 0].max() - points[:, 0].min(), points[:, 1].max() - points[:, 1].min())
                if min_width <= width <= max_width:
                    center = calculate_cluster_center(cluster_pcd)
                    # 이전 중심점과 이동 거리 비교
                    matched_id, distance = match_clusters(previous_centers, center)
                    if matched_id and min_movement_distance < distance <max_movement_distance:  # 이동 기준 충족
                        bbox = cluster_pcd.get_oriented_bounding_box()
                        bbox.color = (1, 0, 0)
                        bboxes.append(bbox)
                        vis.add_geometry(bbox)
                    
                    temp_center[i] = center
    
    previous_centers = temp_center

    # 카메라 뷰 복원
    view_control.convert_from_pinhole_camera_parameters(camera_params)
    vis.update_geometry(downsample_pcd)
    vis.poll_events()
    vis.update_renderer()
    # 프레임을 이미지로 캡처하여 영상으로 저장
    frame_image_path = os.path.join(output_folder, f"frame_{frame:04d}.png")
    vis.capture_screen_image(frame_image_path)

    # OpenCV를 사용하여 영상을 저장
    frame_image = cv2.imread(frame_image_path)
    video_writer.write(frame_image)
    frame+=1

video_writer.release()
frame_end = time.time()
total_time = frame_end - frame_start
average_time = total_time / frame
print(f"Total time: {total_time:.2f}s")
print(f"Average time per frame: {average_time:.2f}s")

# Visualizer 종료
vis.destroy_window()