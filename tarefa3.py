import cv2
import numpy as np
import open3d as o3d
import os
import copy
from scipy.optimize import least_squares
# Removidas as importações de GUI/Rendering para máxima estabilidade

############################################################
#                Pré Processamento
############################################################

# Caminhos para as pastas (AJUSTAR NO SEU AMBIENTE)
base_path = "tum_dataset"
# EXEMPLO: Estes caminhos DEVEM ser ajustados para o seu computador
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
    # Carrega RGB e Depth
    color_cv = cv2.cvtColor(cv2.imread(rgb_file), cv2.COLOR_BGR2RGB)
    depth_cv = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)

    if depth_cv is None or color_cv is None:
        raise FileNotFoundError(f"Erro a ler {rgb_file} ou {depth_file}")

    if depth_cv.dtype != np.uint16:
        depth_cv = depth_cv.astype(np.uint16)

    # Converter para formato Open3D
    color_raw = o3d.geometry.Image(color_cv)
    depth_raw = o3d.geometry.Image(depth_cv)
    
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color=color_raw,
        depth=depth_raw,
        depth_scale=5000.0, 
        depth_trunc=4.0, 
        convert_rgb_to_intensity=False
    )

    # Cria nuvem de pontos
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        camera_intrinsics
    )

    # Corrigir orientação
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])

    # Diminui a intensidade e estima normais
    pcd_down = pcd.voxel_down_sample(voxel_size=0.02)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
    )

    return pcd_down

# Exemplo: processar imagens 1 e 2
print("A processar nuvens de pontos 1 e 2...")
pointcloud_fonte = process_rgbd_pair(os.path.join(rgb_path, "1.png"),
                             os.path.join(depth_path, "1.png"))
pointcloud_alvo = process_rgbd_pair(os.path.join(rgb_path, "2.png"),
                             os.path.join(depth_path, "2.png"))


############################################################
#                TAREFA 3 — Esfera Englobante Mínima
############################################################

# --- CONSTANTES PARA FORÇAR A RESTRIÇÃO RÍGIDA ---
PENALTY = 1e6 
EPSILON = 1e-6

def Erro_Esfera(vetor_esfera, combined_points):
    
    #Função objetivo com Penalização Reforçada para forçar a contenção estrita.
 
    xc, yc, zc, r = vetor_esfera
    center = np.array([xc, yc, zc])   #centro da esfera

    # 1. Distância de cada ponto ao centro: d_i = || p_i - c ||
    distances = np.linalg.norm(combined_points - center, axis=1)  #distancia euclidiana; axis=1 vê linha a linha 

    # 2. Resíduos para as Restrições (Penalização Reforçada)
    # np.maximum(0, x) devolve 0 se x < 0 (ponto dentro da esfera), ou o valor de x se for positivo (ponto fora) e multiplica-o pela penalização
    constraint_residuals = np.maximum(0, distances - r) * PENALTY 

    # 3. Resíduo para o Objetivo (Minimizar r)
    objective_residual = np.array([r])   #raio em array só para estar compatível com o least squares 

    return np.hstack((objective_residual, constraint_residuals))  #array 1D


# --- 1. Preparação dos Dados e Parâmetros Iniciais ---

transf =  np.array([
    [ 0.98073911, -0.07599905,  0.17993039,  0.91113723],
    [ 0.07860547,  0.99687841, -0.00738975,  0.06264181],
    [-0.1788071 ,  0.02139093,  0.98365159, -0.01373347],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

pointcloud_fonte.transform(transf)

# Combinação de pontos das duas nuvens
source_points = np.asarray(pointcloud_fonte.points)
target_points = np.asarray(pointcloud_alvo.points)
combined_points = np.vstack((source_points, target_points))
print(f"Total de pontos para envolver: {combined_points.shape[0]}")


# Inicialização dos Parâmetros (x0)
min_coord = np.min(combined_points, axis=0)
max_coord = np.max(combined_points, axis=0)
center_init = (min_coord + max_coord) / 2 # Centro inicial

dists = []
for p in combined_points:
    dist = np.linalg.norm(p - center_init)
    dists.append(dist)

radius_init = max(dists)

x0_esfera = np.append(center_init, radius_init)  #vetor da esfera inicial


# --- 2. Execução da Otimização ---

print("\n--- INÍCIO DA OTIMIZAÇÃO (Esfera Englobante Mínima - Penalidade Reforçada) ---")
print(f"Palpite Inicial: Centro={center_init}, Raio={radius_init:.4f}")

# Otimização com least_squares
res_esfera = least_squares(Erro_Esfera, x0_esfera, args=(combined_points,), method='lm', verbose=1,ftol=1e-8 )


# Extração dos resultados
xc_final, yc_final, zc_final, r_final = res_esfera.x

print("\n--- RESULTADO DA OTIMIZAÇÃO ---")
print(f"Centro Final (xc, yc, zc): ({xc_final:.4f}, {yc_final:.4f}, {zc_final:.4f})")
print(f"Raio Mínimo Final (r): {r_final:.4f} metros")
print(f"Custo Final (r²): {res_esfera.cost:.6f}")

# --- 3. Visualização Estável e Cortada ---

# 2. DEFINIÇÃO DO CORTE ROBUSO (Cortamos X <= XC_FINAL)
# O centro e o raio finais são usados para definir o corte, garantindo que ele funciona.
xc, yc, zc = xc_final, yc_final, zc_final
r = r_final

# 1. Preparação da Esfera
esfera_minima = o3d.geometry.TriangleMesh.create_sphere(radius=r_final)
esfera_minima.translate([xc_final, yc_final, zc_final])
esfera_minima.paint_uniform_color([1.0, 0.0, 0.0]) # Vermelho Sólido



# Criar wireframe da esfera para efeito de transparência
esfera_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(esfera_minima)
esfera_wireframe.paint_uniform_color([1.0, 0.0, 0.0])  # vermelho

# Preparar as nuvens de pontos para visualização
fonte_viz = copy.deepcopy(pointcloud_fonte)
alvo_viz = copy.deepcopy(pointcloud_alvo)
fonte_viz.paint_uniform_color([0.7, 0.7, 0.7]) 
alvo_viz.paint_uniform_color([0.5, 0.5, 0.5])

# Visualização final com esfera wireframe (transparente)
o3d.visualization.draw_geometries(
    [fonte_viz, alvo_viz, esfera_wireframe],
    window_name="Esfera Englobante Mínima - Transparente"
)