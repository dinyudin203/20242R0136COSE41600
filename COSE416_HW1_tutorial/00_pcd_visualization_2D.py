# 시각화에 필요한 라이브러리 불러오기
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import time

# pcd 폴더 불러오기
folder_path = r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd"

def visualize_pcd_2d(file_path, point_size):
    pcd = o3d.io.read_point_cloud(file_path)

    points = np.asarray(pcd.points)

    if (points.size == 0):
        print("No points in the pcd file")
        return
    points_xy = points[:, :2]

    plt.figure(figsize=(10, 10))
    plt.scatter(points_xy[:, 0], points_xy[:, 1], s=point_size, c="blue", alpha=0.6)
    plt.title("2D Projection of Point Cloud (XY plane)")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def visualize_pcd_2d_video(folder_path, point_size, frame_time):

    pcd_files = sorted([os.path.join(folder_path, file) for file in os.listdir(folder_path) if file.endswith(".pcd")])

    if not pcd_files:
        print("No PCD files found in the specified folder.")
        return

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    scatter = None

    for file_path in pcd_files:
        print(f"Loading {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        points = np.asarray(pcd.points)

        if (points.size == 0):
            print("No points in the pcd file")
            continue

        points_xy = points[:, :2]

        if scatter is None:
            scatter = ax.scatter(points_xy[:, 0], points_xy[:, 1], s=point_size, c="blue", alpha=0.6)
            ax.set_title("2D Projection of Point Cloud (XY plane)")
            ax.set_xlabel("X Coordinate")
            ax.set_ylabel("Y Coordinate")
            ax.axis("equal")
            ax.grid(True)
        else:
            scatter.set_offsets(points_xy)

        plt.pause(frame_time)

    plt.ioff()
    plt.show()
# visualize_pcd_2d(r"E:\Downloads\COSE416_HW1_tutorial\COSE416_HW1_data_v1\data\02_straight_duck_walk\pcd\pcd_000060.pcd", 0.5)
visualize_pcd_2d_video(folder_path, point_size= 0.5, frame_time=0.2)