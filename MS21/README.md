Installation:

First install the following dependencies:

Dependencies:

PCL

    http://pointclouds.org/downloads/linux.html

For Gaussian Process Classification - gpml package

    http://gaussianprocess.org/gpml/code/matlab/release/gpml-matlab-v3.5-2014-12-08.zip


---
Build icp_pacman_pe:

    cd part-icp_pacman_pe/build
    cmake ..
    make
    sudo make install

---
To do pose estimation and recognition use models from:

    https://iis.uibk.ac.at/public/pacman/birmingham_gt_scenes.zip

Password for the .zip files is "pacman".
