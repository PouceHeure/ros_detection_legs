#!/usr/bin/env python3
import rospy
import tensorflow as tf
import numpy as np 

from ros_detection_legs.deep_learning.preprocessing import segmentation
from ros_pygame_radar_2D.msg import PointPolar, RadarPointCloud
from sensor_msgs.msg import LaserScan

class Detector: 

    def __init__(self,topic_scan,topic_radar,model,tolerance=0.8,length_fill=50):
        self._model = model 
        self._tolerance = tolerance
        self._length_fill = length_fill
        self._sub = rospy.Subscriber(topic_scan, LaserScan, self.callback, queue_size=10)
        self._pub = rospy.Publisher(topic_radar, RadarPointCloud, queue_size=10)

    def callback(self,data):
        # get points 
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        points = []
        for i in range(len(data.ranges)): 
            theta = angle_min + angle_increment * i
            r = data.ranges[i]
            points.append([theta,r])

        # get process data 
        lidar_data = segmentation.LidarData(type="prod")
        lidar_data.load_data_from_array(points)
        lidar_data.processing(limite_distance=0.5,limit_cluster_valid=0.8,limit_jump=20,limit_radius=0.30)
        
        # get clusters (points segmented)
        clusters = lidar_data.get_clusters()
        clusters_points = []

        # predict 
        for cluster in clusters: 
            points = cluster.to_array()
            if(len(points) <= self._length_fill):
                clusters_points.append(cluster.to_array() + [[0,0]] * (self._length_fill-len(points)))
            else: 
                clusters_points.append([[0,0]] * self._length_fill)
        
        # extract legs localisation 
        msg = RadarPointCloud()
        y = model.predict(clusters_points)
        
        indices_valid_y = np.where(y >= self._tolerance)[0]
        for indice in indices_valid_y: 
            point = clusters[indice].compute_center()
            point_msg = PointPolar() 
            point_msg.theta = point.theta 
            point_msg.r = point.r 
            msg.points.append(point)

        # publish 
        self._pub.publish(msg)



if __name__ == '__main__': 
    rospy.init_node("detector_node", anonymous=True)

    model = tf.keras.models.load_model('/home/hugo/Documents/projects/ros_detection_legs/model/train')   
    d = Detector("scan","radar",model)
    
    rospy.spin()