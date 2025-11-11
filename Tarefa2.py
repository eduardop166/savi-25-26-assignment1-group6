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
#FUNCAO QUE RECEBE POINT CLOUD FONTE E ALVO E UMA TRANSFORMACAO, APLICA A TRANS AO TARGET E CALCULA DISTANCIAS INDIVIDUAIS E TOTAL 
def transforma_te(pcl_fonte, trans):
    # TRANSFORMA trans (tx, ty, tz, rx, ry, rz) em matriz 4x4
    tx, ty, tz, rx, ry, rz = trans
    # rotação a partir dos ângulos de Euler (XYZ)
    R_mat = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [tx, ty, tz]

    # copiar target para não alterar o original
    pcl_fonte_trans = copy.deepcopy(pcl_fonte)
    pcl_fonte_trans.transform(T.copy())
    
    return pcl_fonte_trans

def Erro(trans ,pcl_fonte, pcl_alvo):

    pcl_fonte_atual = transforma_te(pcl_fonte,trans)
    # Preparar KD-tree
    source_tree = o3d.geometry.KDTreeFlann(pcl_fonte_atual)

    source_points = np.asarray(pcl_fonte_atual.points) # não usado direto, mas útil
    target_points = np.asarray(pcl_alvo.points) 

    correspondencias = [] # LISTA DE CORRESPONDENCIAS (indice da fonte, indice do alvo)
    distancias_ind = [] # LISTA DE DISTANCIAS PARA CADA CORRESPONDENCIA
    dist_total = 0.0 # DISTANCIA TOTAL

    # Para cada ponto da fonte (pc1), encontra o vizinho mais próximo na alvo (pc2)
    for i, p in enumerate(target_points):
        k, idx, dist = source_tree.search_knn_vector_3d(p, 1)
        if k > 0:
            correspondencias.append((i, idx[0]))
            dist_total += dist[0]  # dist[0] já é distância ao quadrado
            distancias_ind.append(dist[0])
    return distancias_ind

### FALTA OTIMIZACAO, VALID DIST( QUANDO DISTANCIA E MENOR QUE THRESHOLD É VALIDO (1) E INVALIDO QUANDO MAIOR (0) )


# CODIGO PARA TESTAR A FUNCAO TRANSFORMA_TE

#[ tx=0.912772, ty=0.071510, tz=-0.009936,  rx=0.017766, ry=0.171394, rz=0.081535 ] dado pelo ICP - Tarefa 1
#trans = [0.912,0.071510,-0.00993,0.0177,0.1713,0.0815]
#a,b,c,pcl_transformada= transforma_te(pointcloud_fonte, pointcloud_alvo, trans)
#print("DISTANCIA TOTAL", a)
#print("CORRESPONDENCIAS",b)
#print("DISTANCIAS INDIVIDUAIS",c)
### FALTA OTIMIZACAO, VALID DIST( QUANDO DISTANCIA E MENOR QUE THRESHOLD É VALIDO (1) E INVALIDO QUANDO MAIOR (0),
# DEPOIS NO CALCULO DO ERRO APENAS SOMAR OS VALIDOS )
#o3d.visualization.draw_geometries([pointcloud_alvo, pointcloud_fonte],window_name="ANTES DO ICP")
#o3d.visualization.draw_geometries([pointcloud_alvo, pcl_transformada],window_name="DEPOIS DO ICP")

transini = [0,0,0,0,0,0]
res = least_squares(Erro, transini, args=(pointcloud_fonte, pointcloud_alvo), method='lm')
print(res.x)





