import os 
import math 
import csv

import matplotlib.pyplot as plt 

PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FOLDER_DATA = os.path.join(PATH_FILE_CURRENT,"../../../../data/")
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

    def get_label(self):
        return self._label

    def add(self,point):
        # keep only point with r > 0 
        if(point.r != 0):
            self._points.append(point)

    def compute_center(self): 
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

    def update_information(self,gamma,type="train"): 
        if(type == "train"):
            self._label = self._define_type(gamma)

    def to_array(self):
        array = []
        for p in self._points: 
            array.append([p.theta,p.r])
        return array 

class LidarData: 
    
    def __init__(self,type="train"): 
        self._type = type

        self._points = []
        self._clusters = []

    def get_points(self):
        return self._points

    def get_clusters(self):
        return self._clusters

    def load_data_from_csv(self,file_path): 
        points = []
        with open(file_path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in spamreader:
                theta = float(row[0])
                r = float(row[1])
                selected = None
                if(self._type == "train"): 
                    selected = int(row[2])
                points.append(PointPolar(theta,r,selected))
        self._points = points

    def load_data_from_array(self,array):
        points = [] 
        for row in array:
            theta = float(row[0])
            r = float(row[1])
            selected = None
            if(self._type == "train"): 
                selected = int(row[2])
            points.append(PointPolar(theta,r,selected))
        self._points = points

    def processing(self,limite_distance=1,limit_cluster_valid=0.8,limit_jump=5,limit_radius=0.5): 
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
                    if(distance < limite_distance and radius_delta < limit_radius): 
                        points_jump = 0 
                        current_cluster.add(p_compare)
                        points.remove(p_compare)
                    else: 
                        i += 1 
                        points_jump += 1 
                current_cluster.update_information(limit_cluster_valid,self._type)

    def generate_dataset(self,type="train"):
        X = []
        y = []
        for cluster in self._clusters:
            points = []
            for p in cluster._points: 
                points.append("%".join(list(map(str,[p.theta,p.r]))))
            X.append(points)
            if(self._type == "train"):
                y.append(cluster.get_label())
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
            if(cluster.get_label() == 1): 
                alpha = 1
            ax.scatter(thetas,rs,alpha=alpha)
        plt.show()

    def __repr__(self):
        return str(self._points)

