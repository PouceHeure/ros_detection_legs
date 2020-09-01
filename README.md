# ros_detection_legs 

![tag:status:status:development](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/status/status_development/status_development_red.png)

![tag:category:deep-learning](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/category/deep-learning/deep-learning_blue.png)
![tag:category:robotic](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/category/robotic/robotic_blue.png)
![tag:lib:ROS](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/lib/ROS/ROS_blue.png)
![tag:language:python3](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/language/python3/python3_blue.png)
![tag:lib:tensorflow](https://raw.githubusercontent.com/PouceHeure/markdown_tags/v1.0/tags/lib/tensorflow/tensorflow_blue.png)

- [ros_detection_legs](#ros_detection_legs)
  - [goal](#goal)
  - [use](#use)
    - [deep-learning](#deep-learning)
      - [prepocessing data](#prepocessing-data)
      - [train model](#train-model)
    - [ros](#ros)
      - [compilation packages](#compilation-packages)
      - [run nodes](#run-nodes)
  - [architecture](#architecture)
    - [extract data](#extract-data)
    - [preprocessing data](#preprocessing-data)
      - [segmentation](#segmentation)
      - [increase positive data](#increase-positive-data)
    - [training](#training)
    - [prediction](#prediction)

## goal 
Extract legs positions from lidar data, like this: 

[![youtube_presentation](./.doc/img/screen-video.png)](https://www.youtube.com/watch?v=KcfxU6_UrOo)

https://www.youtube.com/watch?v=KcfxU6_UrOo

## use

### deep-learning

:warning: a model is already trained, saved in **./model/** folder 
:pencil: if you want to change some parameters, please update [./src/ros_detection_legs/deep_learning/config/parameters.json](./src/ros_detection_legs/deep_learning/config/parameters.json)
#### prepocessing data

run prepocessing script:
```
$ python3 src/preprocessing.py
```
once run data are stored in **./data/train/**

#### train model

run training script: 
```
$ python3 src/training.py 
```

### ros

#### compilation packages 

```
# clone 'ros_pygame_radar_2D' to your workspace
$ git clone https://github.com/PouceHeure/ros_pygame_radar_2D

# compile package 
$ catkin build ros_pygame_radar_2D
```

```
# git 'ros_detection_legs' (this pkg) to your workspace
$ git clone https://github.com/PouceHeure/ros_detection_legs

# inside your ros workingspace
$ catkin build ros_detection_legs
```

#### run nodes 

:warning: source devel before


1. run [detector_node](./nodes/detecor_node.py) 
```
$ rosrun ros_detection_legs detector_node.py
```

2. use [ros_pygame_radar2D](https://github.com/PouceHeure/ros_pygame_radar_2D)
```# run radar_node.py 
$ rosrun ros_pygame_radar_2D radar_node.py
```

3. update parameters, some parameters about model are dynamically reconfigurable. By default **dynamic_reconfigure** loads settings save in parameters.json, don't forget to recompile the project if you change parameters.json !
- compile cfg and generate new configuration file: 
```
# update parameters.json 
$ catkin build 
```
- update parameters dynamically: 
```
$ rosrun rqt_reconfigure rqt_reconfigure
```

![screen-rqt_dyn](.doc/img/screen-rqt_dyn.png)

## architecture

### extract data
- package: **ros_lidar_recorder** https://github.com/PouceHeure/ros_lidar_recorder
- data labeling tool: **lidar_tool_label** https://github.com/PouceHeure/lidar_tool_label
- dataset: https://github.com/PouceHeure/dataset_lidar2D_legs

![graph_data_acquisition](.doc/graph/data_acquisition.png)

### preprocessing data

This schema defines princpals steps: 

![graph_processing](.doc/graph/prepocessing_steps.png)

We can create clusters with a segmentation method, by this way data will gathering if there are closed. 

#### segmentation

Before segmentation, we have to found a way how to compute a distance between 2 polar points. 

- First approach, convert all polar points to cartesien and apply a classic norm, but the complexity of this method is too high. 

- Second approach, find a general expression from polar point to cartesian distance. 

<!-- $
distance(p1,p2) = \sqrt{r_1^2*r_2^2 - 2*_1*r_2*cos(\theta_1-\theta_2)}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/distance/VE9553nyWt.svg"/>


A second distance is computed: 

<!-- $
distance_{radius}(p1,p2) = abs(r_1 - r_2)
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/distance_radius/9VFuqCi7K1.svg"/>


Once expressions are defined, we have to define hyper-parameters:  

- **limit_distance** 
- **limit_radius**

After segmentation, we have to attach a label to each cluster. For this one, we define **limit_cluster_valid**. 

<!-- $
average\_label_{cluster_i} = \frac{card(points_{cluster_i} \text{where point == selected)}}{card(points_{cluster_i})}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/label_definition/RdtwNciQva.svg"/>

<!-- $
label_{cluster_i} =  \left\{
    \begin{array}{ll}
        1 & average\_label_{cluster_i} >= limit\_cluster\_valid \\
        0 & \text{else}
    \end{array}
\right.
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/label_definition/uWssGgYcJR.svg"/>

![graph_segmenation](.doc/graph/segmentation.png)


#### increase positive data 

We can increase positive data by applying a rotation on these data. 

![graph_raising](.doc/graph/raising.png)

All positive clusters are selected, on each cluster the same tranformation is applied on each point.   

The transformation is done by this expression: 

<!-- $
\theta' = \theta + \theta_{transformation}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/transformation/gZTVMPSyj7.svg"/>
 
So at after transformation, if we have *N* transformations

<!-- $
size_{dataset\_initial} = card(dataset_{initial})
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/transformation/ck675XtbLJ.svg"/>

<!-- $
size_{positive\_data} = card(dataset_{initial} \text{where y == 1})
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/transformation/hxiBg4rXbC.svg"/>

<!-- $
size_{dataset\_final} = size_{dataset\_initial} + N * size_{positive\_data}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/transformation/C48mkl3bJQ.svg"/>


### training 

model used: RNN with LSTM cells. 

more information about LSTM: https://www.tensorflow.org/api_docs/python/tf/keras/layers/LSTM

### prediction

A ros node, **detector_node** subscribes to **/scan** topic. Once data are pusblished to this topic, the node uses the training model to predict legs positions. Legs positions are published to **/radar** topic. 

![graph_prediction](.doc/graph/prediction_ros.png)

Like the training, data need to be tranformed. So before prediction, clusters are created directly in the subscriber callback function. 

![graph_prediction](.doc/graph/prediction.png)

Define the center point of positive clusters: 

![graph_locate_center_point](./doc/../.doc/graph/locate_center_point.png)

Coordinate of the center of the cluster *j* are:

<!-- $
\theta_{j_{center}} = \frac{1}{card(points_j)}\sum_{i} \theta_{i}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/center/hBFtPnjogX.svg"/>

<br/>

<!-- $
r_{j_{center}} = \frac{1}{card(points_j)}\sum_{i} r_{i}
$ --> <img style="transform: translateY(0.25em);" src=".doc/equation/center/WifGokqCvm.svg"/>