# Part-based Planner
COPYRIGHT BY RENAUD DETRY 2015!!

## Install

First, install [Nuklei](http://nuklei.sourceforge.net/doxygen/group__install.html).

Third, compile pbp by typing

```
make
sudo make install
```

## FOR VISUALIZATION PLEASE USE NUKLEI TO CONVERT THE POINTCLOUDS TO PCD FILES AND USE PCL_VIEWER 


## pbp-planner.py

The script pbp-planner.py implements a high-level planner functionality.

Usage:

```
pbp-planner.py -d DIRECTORY
```

where DIRECTORY is the name of a directory directly contained in the directory `pbp-planner` is called from, and contains a file `image-0.pcd` that represents a point cloud.

For instance,

```
pbp-planner.py -d experiment1
```


By default, `pbp-planner.py` expects `image-0.pcd` to be of PCD type, and it performs a number of transformations on the file before planning the grasp. These transformations include cropping, scaling, dominant plane removal, 3D translation/rotation (to go from the camera frame to the robot frame), and providing a point-cloud representation of obstacles. These transformations expect three files to be present in the directory where `pbp-planner.py` is called from: `workspace.inkinect.m.txt`, `from-kinect-to-robot.txt`, and `obst.pcd`,  which parametrize cropping and 3D transformation, and define extra obstacles. If you do not wish to perform any of the operations above (e.g., if your point cloud is ready to go), call

```
pbp-planner.py -d DIRECTORY --config config.cfg
```

where `config.cfg` looks like

```
[pbp-config]
input_name: image-0.pcd
autodetect_input_type: false
input_type: pcd
crop_pcd_file: true
cropbox_file: workspace.inkinect.m.txt
scale: 1000
remove_dominant_plane: true
transform: true
transformation_file: from-kinect-to-robot.txt
extra_obstacles: true
extra_obstacles_file: obst.pcd
n_prototypes: 2
min_dist: 3
pe_position_bandwidth: 10
```

The options given here are the default ones. Change according to your likes. Note that `cropbox_file`, `transformation_file`, and `extra_obstacles_file` must be provided relatively to the directory `pbp-planner.py` is called from.
