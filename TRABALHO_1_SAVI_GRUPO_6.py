import cv2
import numpy as np
import open3d as o3d
import os

# Caminhos para as pastas 
base_path = "tum_dataset"
#rgb_path = r"C:\Users\Eduardo Pereira\Documents\UNI\A2-S1\SAVI\TRABALHO1\tum_dataset\rgb"
#depth_path = r"C:\Users\Eduardo Pereira\Documents\UNI\A2-S1\SAVI\TRABALHO1\tum_dataset\depth"

rgb_path = r"C:\Universidade\Mestrado\2o Ano\SAVI\savi-25-26-assignment1-group6\tum_dataset\rgb"
depth_path = r"C:\Universidade\Mestrado\2o Ano\SAVI\savi-25-26-assignment1-group6\tum_dataset\depth"

# Parâmetros intrínsecos da camera
fx, fy = 525.0, 525.0
cx, cy = 319.5, 239.5
camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
camera_intrinsics.set_intrinsics(width=640, height=480, fx=fx, fy=fy, cx=cx, cy=cy)

def process_rgbd_pair(rgb_file, depth_file):
    # Carrega RGB e Depth (OpenCV garante tipos corretos)
    color_cv = cv2.cvtColor(cv2.imread(rgb_file), cv2.COLOR_BGR2RGB)
    depth_cv = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)

    if depth_cv is None or color_cv is None:  
        raise FileNotFoundError(f"Erro a ler {rgb_file} ou {depth_file}")

    # Garantir que esta em uint16
    if depth_cv.dtype != np.uint16:
        depth_cv = depth_cv.astype(np.uint16)

    # Converter para formato Open3D
    color_raw = o3d.geometry.Image(color_cv)
    depth_raw = o3d.geometry.Image(depth_cv)
    
#    # Cria RGBDImage (nao mantem as cores) (nao sei porque o prof usou este no ex da aula se fica sem cores)
#    rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
#        color_raw,
#        depth_raw
#    )

    # Cria RGBDImage (mantendo as cores RGB)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color=color_raw,
        depth=depth_raw,
        depth_scale=5000.0,   # padrão TUM, valores do depth vêm em milimetros 
        depth_trunc=4.0,      # corta profundidades > 4 m
        convert_rgb_to_intensity=False  # mantém cores reais
    )

    # Cria nuvem de pontos
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        camera_intrinsics
    )

    # Corrigir orientação (Open3D → OpenCV) (usam diferentes sistemas de coordenadas)
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])

    # Diminui a intensidade 
    pcd_down = pcd.voxel_down_sample(voxel_size=0.02)

    # Estima normais
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
    )

    return pcd_down

# Exemplo: processar imagens 1 e 2
pcd1 = process_rgbd_pair(os.path.join(rgb_path, "1.png"),
                         os.path.join(depth_path, "1.png"))
pcd2 = process_rgbd_pair(os.path.join(rgb_path, "2.png"),
                         os.path.join(depth_path, "2.png"))

# Visualização
o3d.visualization.draw_geometries([pcd2])
