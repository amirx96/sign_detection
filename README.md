# Sign Detection
Howdy! 🤠

This is a little package I made while in Undergraduate to perform some basic intensity filtering from a LIDAR PointCloud2 message to obtain a filtered pointcloud of street signs.
### Video:

[![Youtube Video](https://img.youtube.com/vi/b7DnjrF072w/0.jpg)](https://youtu.be/b7DnjrF072w)

# Requirements
* ROS-Kinetic (untested with Melodic)
* PCL-ROS

## Setup
Git clone to your `catkin_ws`

Then

```bash
cd catkin_ws
rosdep install rosdep install --from-paths src --ignore-src -r -y
```

## LIDAR
This package was tested with the Velodnye VLP-16 LIDAR. With the VLP-16, I was reliabily able to detect stop signs from up to 10 meters away. 

In theory, any other LIDAR that outputs a Pointcloud2 message with intensity should work, provided that there is enough points.


## Subscribed Topics

* `/velodyne_points`

## Published Topics
* `/sd_pointcloud_filtered` -- Filtered Pointcloud2 message of the sign
* `/sd_distance` -- Distance to the sign
* `/visualization_marker` -- Visualation Marker message of the sign for RViz

