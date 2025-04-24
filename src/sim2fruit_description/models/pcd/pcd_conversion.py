# convert from .txt -> .ply -> .dae

import open3d as o3d
import numpy as np
import aspose.threed as a3d
import os

TXT_INPUT = "./tomato_plant.txt"
PLY_OUTPUT = "./tomato_plant.ply"
DAE_OUTPUT = "../tomato/meshes/tomato_plant.dae"

def txt_to_ply(txt_path, ply_path):
    points = np.loadtxt(txt_path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(ply_path, pcd)

def ply_to_dae_aspose(ply_path, dae_path):
    scene = a3d.Scene.from_file(ply_path)
    scene.save(dae_path)

def main():
    txt_to_ply(TXT_INPUT, PLY_OUTPUT)
    ply_to_dae_aspose(PLY_OUTPUT, DAE_OUTPUT)

if __name__ == "__main__":
    main()
