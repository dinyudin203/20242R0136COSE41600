import open3d as o3d
import os

# PCD 파일이 저장된 폴더 경로들
folder_paths = [
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/01_straight_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/02_straight_duck_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/03_straight_crawl/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/04_zigzag_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/05_straight_duck_walk/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/06_straight_crawl/pcd',
    'E:/Downloads/COSE416_HW1_tutorial/COSE416_HW1_data_v1/data/07_straight_walk/pcd',
]

# 현재 스크립트 실행 경로
script_dir = os.path.dirname(os.path.abspath(__file__))

# 저장 폴더 경로 생성
output_folder = os.path.join(script_dir, 'pcd')
os.makedirs(output_folder, exist_ok=True)  # 폴더가 없으면 생성

for folder_path in folder_paths:
    # 읽을 파일 경로와 저장할 파일 경로
    file_path = os.path.join(folder_path, 'pcd_000001.pcd')
    folder_name = os.path.basename(os.path.normpath(folder_path))
    output_file_path = os.path.join(output_folder, f"{folder_name}_pcd_000001_ascii.pcd")

    # PCD 파일 읽기
    if os.path.exists(file_path):  # 파일 존재 여부 확인
        temp_pcd = o3d.io.read_point_cloud(file_path)
        
        # 텍스트 형식으로 저장
        o3d.io.write_point_cloud(output_file_path, temp_pcd, write_ascii=True)
        print(f"Saved ASCII PCD: {output_file_path}")
    else:
        print(f"File not found: {file_path}")
