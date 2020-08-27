import os 
import math 
import csv

import matplotlib.pyplot as plt 



PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FOLDER_DATA = os.path.join(PATH_FILE_CURRENT,"../../data/")
PATH_FOLDER_DATA_PROCESSED = os.path.join(PATH_FOLDER_DATA,"processed/")
PATH_FOLDER_DATASET_LIDAR = os.path.join(PATH_FOLDER_DATA,"dataset_lidar2D_legs/")
PATH_FOLDER_DATASET_LIDAR_50cm = os.path.join(PATH_FOLDER_DATASET_LIDAR,"50cm/")


class PointPolar: 

    COUNTER_ID = 0

    def __init__(self,theta,r,selected=None): 
        self.theta = theta 
        self.r = r 
        self.selected = selected

        self.id = PointPolar.COUNTER_ID
        PointPolar.COUNTER_ID += 1

    @staticmethod
    def COMPUTE_DISTANCE(p1,p2):
        return math.sqrt(p1.r**2+p2.r**2 - 2*p1.r*p2.r*math.cos(p1.theta-p2.theta))

    @staticmethod
    def COMPUTE_RADIUS(p1,p2):
        return abs(p1.r - p2.r)

    def __repr__(self):
        return f"[id: {self.id}] (theta: {self.theta} r: {self.r})"# - selected: {self.selected}"


class Cluster: 
    
    COUNTER_ID = 0

    def __init__(self): 
        self._points = []

        self.id = Cluster.COUNTER_ID
        Cluster.COUNTER_ID += 1 

    def get_points(self):
        return self._points

    def add(self,point):
        self._points.append(point)

    def _compute_center(self): 
        theta_sum = 0
        r_sum = 0 
        n = len(self._points)*1.
        for p in self._points: 
            theta_sum += p.theta
            r_sum += p.r 
        return PointPolar(theta_sum/n,r_sum/n)

    def _define_type(self,gamma): 
        n = len(self._points)*1.
        n_selected = 0
        for p in self._points: 
            n_selected += p.selected
        if(n_selected/n >= gamma): 
            return 1
        return 0 

    def update_information(self,gamma): 
        self.center = self._compute_center()
        self.label = self._define_type(gamma)


class FileData: 
    
    def __init__(self,file_path,file_type="train"): 
        self._points = self._load_data(file_path,file_type)
        self._clusters = []

    def _load_data(self,file_path,file_type): 
        points = []
        with open(file_path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in spamreader:
                theta = float(row[0])
                r = float(row[1])
                selected = None
                if(file_type == "train"): 
                    selected = int(row[2])
                points.append(PointPolar(theta,r,selected))
        return points

    def get_points(self):
        return self._points

    def processing(self,epsilon=1,gamma=0.8,limit_jump=5,limit_radius=0.5): 
        self._clusters = []
        points = self._points
        while(len(points) != 0): 
            p = points[0]
            points.remove(p)
            if(p.r != 0):
                current_cluster = Cluster()
                self._clusters.append(current_cluster)
                current_cluster.add(p)
                points_jump = 0
                i = 0 
                while(i < len(points) and points_jump < limit_jump):
                    p_compare = points[i]
                    distance = PointPolar.COMPUTE_DISTANCE(p,p_compare)
                    radius_delta = PointPolar.COMPUTE_RADIUS(p,p_compare)
                    if(distance < epsilon and radius_delta < limit_radius): 
                        points_jump = 0 
                        current_cluster.add(p_compare)
                        points.remove(p_compare)
                    else: 
                        i += 1 
                        points_jump += 1 
                current_cluster.update_information(gamma)

    def generate_dataset(self,type="train"):
        X = []
        y = []
        for cluster in self._clusters:
            points = []
            for p in cluster._points: 
                points.append("%".join(list(map(str,[p.theta,p.r]))))
            X.append(points)
            if(type == "train"):
                y.append(cluster.label)
        return X,y

    def plot_clusters(self): 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')
        for cluster in self._clusters: 
            thetas = []
            rs = []
            for p in cluster.get_points(): 
                thetas.append(p.theta)
                rs.append(p.r)
            alpha=0.1
            if(cluster.label == 1): 
                alpha = 1
            ax.scatter(thetas,rs,alpha=alpha)
        plt.show()

    def __repr__(self):
        return str(self._points)


def load_paths_files(folder_root):
    paths_files = []
    for file_name in os.listdir(folder_root): 
        file_path = os.path.join(folder_root, file_name)
        if os.path.isfile(file_path) and ".csv" in file_name:
            paths_files.append(file_path)
    return paths_files


def keep_proportion(X,y): 
    number_y_to_1 = y.count(1)
    new_X = []
    new_y = []
    number_y0_added = 0
    for i in range(len(X)): 
        if(len(X[i]) < 50):
            if(y[i] == 1 or (y[i] == 0 and number_y0_added < 2*number_y_to_1)):
                new_X.append(X[i])
                new_y.append(y[i]) 
                number_y0_added += 1-y[i] 

    return new_X,new_y


def create_dataset_train(folder_in,file_out):
    paths_files = load_paths_files(folder_in) 
    X = []
    y = []
    for path in paths_files: 
        filedata = FileData(path,file_type="train")
        filedata.processing(epsilon=0.5,gamma=0.8,limit_jump=20,limit_radius=0.30)
        X_, y_ = filedata.generate_dataset(type="train")
        X += X_
        y += y_

    X,y = keep_proportion(X,y)

    with open(file_out, 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            for i in range(len(X)): 
                spamwriter.writerow([y[i]] + X[i])
        

    


if __name__ == "__main__":  
    create_dataset_train(PATH_FOLDER_DATASET_LIDAR_50cm,os.path.join(PATH_FOLDER_DATA_PROCESSED,"train.csv"))