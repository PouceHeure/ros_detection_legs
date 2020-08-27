#!/usr/bin/env python3
import rospy
import tensorflow as tf 

from ros_detection_legs.deep_learning.preprocessing.modules import segmentation
from sensor_msgs.msg import LaserScan

model = None

def callback(data):
    # get points 
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    points = []
    for i in range(len(data.ranges)): 
        theta = angle_min + angle_increment * i
        r = data.ranges[i]
        points.append([theta,r])

    # get lidar 
    lidar_data = segmentation.LidarData(type="prod")
    lidar_data.load_data_from_array(points)
    lidar_data.processing(epsilon=0.5,gamma=0.8,limit_jump=20,limit_radius=0.30)
    
    # get clusters
    clusters = lidar_data.get_clusters()
    
    # check each cluster
    clusters_points = []
    for cluster in clusters: 
        clusters_points.append(cluster.to_array())

    if(len(clusters_points) != 0):
        model.predict(clusters_points)
    
        

if __name__ == '__main__': 
    rospy.init_node("detector_node", anonymous=True)


    model = tf.keras.models.load_model('/home/hugo/Documents/projects/ros_detection_legs/model/train')   

    rospy.Subscriber("scan", LaserScan, callback, queue_size=10)
    
    rospy.spin()