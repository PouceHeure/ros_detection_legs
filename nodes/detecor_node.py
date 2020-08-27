#!/usr/bin/env python3
import rospy
import tensorflow as tf 

from ros_detection_legs.deep_learning.preprocessing.modules import segmentation
from ros_pygame_radar_2D.msg import PointPolar, RadarPointCloud
from sensor_msgs.msg import LaserScan

model = None
pub = None 

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
    
    # for cluster in clusters: 
    #     points = cluster.to_array()
    #     y_pred = model.predict([points])[0][0]
    #     if(y_pred > 0.80): 
    #         point = cluster.get_center()
    #         point_msg = PointPolar()
    #         point_msg.theta = point.theta 
    #         point_msg.r = point.r 
    #         msg.points.append(point)
    
    clusters_points = []
    n = 50
    for cluster in clusters: 
        points = cluster.to_array()
        if(len(points) <= n):
            clusters_points.append(cluster.to_array() + [[0,0]] * (n-len(points)))
        else: 
            clusters_points.append([[0,0]] * n)
        
    msg = RadarPointCloud()
    y = model.predict(clusters_points)
    for i in range(len(y)): 
        v = y[i]
        if(v > 0.9):
            point = clusters[i].get_center()
            point_msg = PointPolar()
            point_msg.theta = point.theta 
            point_msg.r = point.r 
            msg.points.append(point)

        # if(y_pred > 0.80): 
        #     point = cluster.get_center()
        #     point_msg = PointPolar()
        #     point_msg.theta = point.theta 
        #     point_msg.r = point.r 
        #     msg.points.append(point)

    pub.publish(msg)


    
        

if __name__ == '__main__': 
    rospy.init_node("detector_node", anonymous=True)


    model = tf.keras.models.load_model('/home/hugo/Documents/projects/ros_detection_legs/model/train')   
    pub = rospy.Publisher('radar', RadarPointCloud, queue_size=1000)

    rospy.Subscriber("scan", LaserScan, callback, queue_size=10)
    
    rospy.spin()