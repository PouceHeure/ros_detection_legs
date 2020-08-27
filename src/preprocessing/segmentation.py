import os 
import math 

PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FOLDER_DATA = os.path.join(PATH_FILE_CURRENT,"../../data/")
PATH_FOLDER_DATASET_LIDAR = os.path.join(PATH_FOLDER_DATA,"dataset_lidar2D_legs/")
PATH_FOLDER_DATASET_LIDAR_50cm = os.path.join(PATH_FOLDER_DATASET_LIDAR,"50cm/")

def compute_distance_polar_points(theta1,r1,theta2,r2):
    return math.sqrt(r1**2+r2**2 - 2*r1*r2*math.cos(theta1-theta2))

def load_paths_files(folder_root):
    paths_files = []
    for file_name in os.listdir(folder_root): 
        file_path = os.path.join(folder_root, file_name)
        if os.path.isfile(file_path) and ".csv" in file_name:
            paths_files.append(file_path)
    return paths_files


if __name__ == "__main__":  
    paths_files = load_paths_files(PATH_FOLDER_DATASET_LIDAR_50cm)