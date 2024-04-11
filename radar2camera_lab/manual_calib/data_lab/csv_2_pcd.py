# -*- coding: utf-8 -*-
import pandas as pd
import open3d as o3d

def csv_to_ply(csv_path, ply_path):
    # 读取 CSV 文件，手动指定列名
    # df = pd.read_csv(csv_path, header=None, names=['x', 'y', 'z'])
    df = pd.read_csv(csv_path, header=None, usecols=[0, 1, 2], names=['x', 'y', 'z'])

    # 创建 Open3D 点云对象
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(df[['x', 'y', 'z']].values)

    # 保存为 PLY 文件
    o3d.io.write_point_cloud(ply_path, cloud, write_ascii=True)

    print('PLY file saved successfully at {}'.format(ply_path))


# 替换为你的 CSV 文件路径和输出 PCD 文件路径
csv_file_path = 'radar_1.csv'
ply_file_path = 'output_cloud.ply'

csv_to_ply(csv_file_path, ply_file_path)