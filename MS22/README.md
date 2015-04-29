Installation:

First install the following dependencies:

Dependencies:

PCL
-http://pointclouds.org/downloads/linux.html

Geometric Tools
-http://www.geometrictools.com/Downloads/WildMagic5p13_k82Z1pAr.zip

Nuklei
-http://nuklei.sourceforge.net/doxygen/

For Gaussian Process Classification, gpml package
-http://gaussianprocess.org/gpml/code/matlab/release/gpml-matlab-v3.5-2014-12-08.zip

For part-based-planner:
-ros hydro
-all packages related to moveit and kdl

---

Call shell script in template_generator/pacman_pilot/download_extract_data.sh

-- build part-based-planner:

cd part-based-planner/build
cmake ..
make
sudo make install

-- to generate templates from scratch (grasp_db, recognized_parts are hardcoded)

cd template_generator/build
cmake ..
make
./template_generator

--to run planner:
see compute_grasps.sh