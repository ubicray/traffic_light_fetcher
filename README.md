
# traffic_light_fetcher

## Overview

A simple package for fetching and performing simple analysis on traffic lights in a streamed video


**Author: Elmar Abbasov<br />
Maintainer: abbasov.elmar94@gmail.com**

The traffic_light_fetcher package has been tested under ROS Kinetic and Ubuntu 16.04.


![enter image description here](https://scontent-hel2-1.xx.fbcdn.net/v/t1.15752-9/84266028_197730321321576_7634645659626242048_n.png?_nc_cat=105&_nc_sid=b96e70&_nc_ohc=t7Tm2fvWbjQAX9h3cGm&_nc_ht=scontent-hel2-1.xx&oh=b74a0188d130ddb2511040fa70d2df5d&oe=5E968FDB)
![enter image description here](https://scontent-hel2-1.xx.fbcdn.net/v/t1.15752-9/89080379_491198138488500_5756183859511164928_n.png?_nc_cat=108&_nc_sid=b96e70&_nc_ohc=lwUXtdjbLpoAX_x3Iv6&_nc_ht=scontent-hel2-1.xx&oh=3c525d14e17f3e35e3291b4f4235a3f5&oe=5E96921C)



#### Non-standard dependencies

- [video_stream_opencv] (to publish a video stream)

		sudo apt install ros-kinetic-video-stream-opencv


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ubicray/traffic_light_fetcher
	cd ../
	catkin build traffic_light_fetcher



## Usage

Before using you'll need to add a video and [pre-trained tensorflow model] into a folder called `include` in the package's directory
For ease of use you can download this archive which has a pre-trained model and the video I tested this package with included - [Google drive link](https://drive.google.com/open?id=1xqNF5Y6TR5BXhMknmrF_ehL4f_Vj8NTe)

Run the main node with

	roslaunch traffic_light_fetcher run_all.launch


## Launch files

* **run_all.launch:** For running all the nodes for seeing how the package works


## Nodes

### traffic_light_fetcher

Grabs images from a streamed image and extracts details about traffic lights in the picture


#### Subscribed Topics

* **`/videofile/image_raw`** (sensor_msgs/Image)


#### Published Topics

* **`/traffic_light_detected`** (std_msgs/Bool)

* **`/traffic_light_size`** (geometry_msgs/Vector3)



### traffic_light_analyzer

#### Subscribed Topics

* **`/traffic_light_detected`** (std_msgs/Bool)

* **`/traffic_light_size`** (geometry_msgs/Vector3)


#### Published Topics

* **`/zone_height`** (std_msgs/Float32)

## Configuration tested with real-time performance

 - Ubuntu 16.04
 - ROS kinetic
 - Nvidia drivers 418.87
 - i7-7700
 - GTX 1070
 - CUDA 10.1

## References
[https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API](https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API)
[https://www.youtube.com/watch?v=7OtcVQ8H2BQ](https://www.youtube.com/watch?v=7OtcVQ8H2BQ) (Video used for testing)



[video_stream_opencv]: http://wiki.ros.org/video_stream_opencv
[pre-trained tensorflow model]: http://download.tensorflow.org/models/object_detection/ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz
