import cv2
import numpy as np
import open3d as o3d
import os
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import copy

############################################################
#                Pré Processamento
############################################################


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
        depth_scale=5000.0,   # padrão TUM
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
pointcloud_fonte = process_rgbd_pair(os.path.join(rgb_path, "1.png"),
                         os.path.join(depth_path, "1.png"))
pointcloud_alvo = process_rgbd_pair(os.path.join(rgb_path, "2.png"),
                         os.path.join(depth_path, "2.png"))


# Guarda point clouds em .ply
o3d.io.write_point_cloud("pcd1.ply", pointcloud_fonte)
o3d.io.write_point_cloud("pcd2.ply", pointcloud_alvo)


# Visualização
#o3d.visualization.draw_geometries([pointcloud_alvo])



############################################################
#                TAREFA 2 — ICP com Open3D
############################################################



#As point clouds têm números de pontos diferentes 
#FUNCAO QUE RECEBE PONTOS E APLICA UMA TRANSFORMACAO

def vetor_matriz(vetor):
    # TRANSFORMA trans (tx, ty, tz, rx, ry, rz) em matriz 4x4
    tx, ty, tz, rx, ry, rz = vetor
    # rotação a partir dos ângulos de Euler (XYZ)
    R_mat = R.from_euler('xyz', [rx, ry, rz]).as_matrix()

    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [tx, ty, tz]

    return T

def matches(pcl_fonte_atual,pcl_alvo):

    target_tree_alvo = o3d.geometry.KDTreeFlann(pcl_alvo)

    source_points = np.asarray(pcl_fonte_atual.points)
    target_points = np.asarray(pcl_alvo.points) 

    correspondencia = [] # LISTA DE CORRESPONDENCIAS (indice da fonte, indice do alvo)
    threshold = 0.15
    source_points_correspondencias = []
    target_points_correspondencias = []

    # Para cada ponto da fonte (pc1), encontra o vizinho mais próximo na alvo (pc2)
    for i, p in enumerate(source_points):   #i - index
        k, idx, dist  = target_tree_alvo.search_knn_vector_3d(p, 1)
        if dist[0] < threshold**2:
            if k > 0:
                correspondencia.append((i, idx[0]))  
                source_points_correspondencias.append(source_points[i]) 
                target_points_correspondencias.append(target_points[idx])         

    return correspondencia, source_points_correspondencias, target_points_correspondencias

def Erro(vetor,source_points, target_points):
    #aplicar a transformação e calcular os erros 

    #vetor novo para matriz  
    matriz_trans = vetor_matriz(vetor)
    Rotacao = matriz_trans[:3, :3]
    Translacao = matriz_trans[:3, 3]
        
    # Transforma os pontos da fonte: p_s' = R_inc * p_s + t_inc
    fonte_transformada = (Rotacao @ source_points.T).T + Translacao

    #Calcular os erros (distancias individuais)
    distancias_ind = fonte_transformada - target_points

    return distancias_ind


############################################################
#                ICP
############################################################



def icp(pcl_source, pcl_target, vetor):
    #Copia da pcl source 
    source = copy.deepcopy(pcl_source)  

    #Transformar o vetor em matriz
    matriz_transformacao = vetor_matriz(vetor)

    #aplicar transformacao inicial
    source.transform(matriz_transformacao)

    #primeira transformacao para a iteracao
    transformacao_iterativa =  matriz_transformacao

    xo = np.zeros(6) # sempre 0 para a transformacao ser incremental 
    iteracoes = 50
    for i in range(iteracoes):
        #Buscar correspondencias 
        Correspondencias, source_points_correspondencias, target_points_correspondencias = matches(source, pcl_target)

        #Otimizar com least_squares
        res = least_squares(Erro, transini,jac='2-point', args=(source_points_correspondencias, target_points_correspondencias), method='lm', verbose=1)
        print(res.x)
        print("Erro final (custo):", res.cost)

        T_incremental = vetor_matriz(res.x)  #trasnformacao do resultado em matriz

        #aplicar nova transformacao 
        source.transform(T_incremental)
        transformacao_iterativa = T_incremental @ transformacao_iterativa

        if res.cost < 5:
            break

    trans_final = transformacao_iterativa

    return source, trans_final

#Mostrar Resultado
transini = [0,0,0,0,0,0]
pcl_final, trans_final = icp(pointcloud_fonte, pointcloud_alvo, transini)













    

