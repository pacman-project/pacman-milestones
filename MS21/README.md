Installation:

First install the following dependencies:

Dependencies:

PCL
-http://pointclouds.org/downloads/linux.html


For Gaussian Process Classification, gpml package
-http://gaussianprocess.org/gpml/code/matlab/release/gpml-matlab-v3.5-2014-12-08.zip


-- build icp_pacman_pe:

cd part-icp_pacman_pe/build
cmake ..
make
sudo make install

--to do pe and recognition use models from:

Here the ground truth poses and scenes including aligned pcd object models:
https://iis.uibk.ac.at/public/pacman/birmingham_gt_scenes.zip

Software to read .pose (xml) file to quaternion (note that the gt poses are in  mm! .pose file).
https://iis.uibk.ac.at/public/pacman/transform_conversions.zip

Password for the .zip files is "pacman".