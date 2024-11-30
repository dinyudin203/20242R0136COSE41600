# 시각화에 필요한 라이브러리 불러오기
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import time

# pcd 폴더 불러오기
folder_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd"

# pcd 파일 불러오고 시각화하는 함수
def load_and_visualize_pcd(file_path, point_size=1.0):
    # pcd 파일 로드
    pcd = o3d.io.read_point_cloud(file_path)
    print(f"Point cloud has {len(pcd.points)} points.")
    
    # 시각화 설정
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.get_render_option().point_size = point_size
    vis.run()
    vis.destroy_window()

# 영상으로 보기
def load_and_visualize_pcd_video(folder_path, point_size, frame_delay):
    # pcd 파일 로드
    pcd_files = sorted([os.path.join(folder_path, file) for file in os.listdir(folder_path) if file.endswith(".pcd")])
    
    if not pcd_files:
        print("No PCD files found in the specified folder.")
        return
    
    print(f"Found {len(pcd_files)} PCD files.")

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    print(f"Loading {pcd_files[0]}")
    pcd = o3d.io.read_point_cloud(pcd_files[0])
    vis.add_geometry(pcd)

    print(f"Point cloud has {len(pcd.points)} points.")

    for file_path in pcd_files:
        print(f"Loading {file_path}")
        
        new_pcd = o3d.io.read_point_cloud(file_path)

        print(f"Point cloud has {len(new_pcd.points)} points.")

        pcd.points = new_pcd.points

        if new_pcd.has_colors():
            pcd.colors = new_pcd.colors
        if new_pcd.has_normals():
            pcd.normals = new_pcd.normals

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        time.sleep(frame_delay)
    
    vis.destroy_window()



# PCD 파일 불러오기 및 데이터 확인 함수
def load_and_inspect_pcd(file_path):
    # PCD 파일 로드
    pcd = o3d.io.read_point_cloud(file_path)
    
    # 점 구름 데이터를 numpy 배열로 변환
    points = np.asarray(pcd.points)
    
    # 점 데이터 개수 및 일부 점 확인
    print(f"Number of points: {len(points)}")
    print("First 5 points:")
    print(points[:5])  # 처음 5개의 점 출력
    
    # 점의 x, y, z 좌표의 범위 확인
    print("X coordinate range:", np.min(points[:, 0]), "to", np.max(points[:, 0]))
    print("Y coordinate range:", np.min(points[:, 1]), "to", np.max(points[:, 1]))
    print("Z coordinate range:", np.min(points[:, 2]), "to", np.max(points[:, 2]))

def add_color_to_point_cloud(file_path):
    # PCD 파일 로드
    pcd = o3d.io.read_point_cloud(file_path)
    
    # 점 데이터를 numpy 배열로 변환
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals) if pcd.has_normals() else None
    
    # 예시 ambient 데이터 생성 (데이터에서 로드해야 함)
    ambient = np.random.randint(0, 256, size=(len(points),))  # 예제: 랜덤 ambient 데이터
    
    # Ambient 값을 Grayscale 색상으로 변환
    ambient_normalized = ambient / 255.0
    ambient_colors = np.stack([ambient_normalized]*3, axis=-1)  # RGB로 변환
    
    # Normal 값을 RGB 색상으로 변환
    if normals is not None:
        normal_colors = (normals + 1) / 2  # [-1, 1] 범위를 [0, 1]로 정규화
    else:
        normal_colors = np.zeros_like(points)  # 기본값
    
    # Ambient와 Normal 색상을 결합 (가중 평균 사용 가능)
    final_colors = 0.5 * ambient_colors + 0.5 * normal_colors
    
    # Open3D에서 색상 설정
    pcd.colors = o3d.utility.Vector3dVector(final_colors)
    
    # 시각화
    o3d.visualization.draw_geometries([pcd])

# PCD 파일 경로
file_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd\pcd_000060.pcd"

# add_color_to_point_cloud(file_path)

# pcd 시각화 테스트

# pcd_files = sorted([os.path.join(folder_path, file) for file in os.listdir(folder_path) if file.endswith(".pcd")])

# for file_path in pcd_files:
#     load_and_visualize_pcd(file_path, 0.5)
#     load_and_inspect_pcd(file_path)

# load_and_visualize_pcd(folder_path, 0.5, 0.2)

# load_and_visualize_pcd_video(folder_path, 0.3, 0.5)

load_and_visualize_pcd(r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\01_straight_walk\pcd\pcd_0000180.pcd", 0.5)
