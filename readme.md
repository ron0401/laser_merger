# Laser Merger

## Over View

The laser scan topic is 2D point cloud.  
However, obstacle detection requires 3D point cloud.  
3D detection method is used by placing multiple 2D lidars.  
This package merges multiple 2D point clouds into a 3D point cloud.  
It also has the future to remove low height points (mostly the ground) and footprint points (robot body).

![](doc/1.gif)

## Get Start

```
roslaunch laser_merger example.launch
```

