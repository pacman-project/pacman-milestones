#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <algorithm>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

//wild magic
#include <Wm5IntrBox3Box3.h>
#include <Wm5IntrBox3Sphere3.h>

//for system call to nuklei

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */



#include <iostream>
#include <iomanip>
#include "forwardKinematics.h"

//boost
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#define DEBUG_TEMPLATE 1

using namespace std;
using namespace Wm5;

//SETTINGS
bool use_registered_clouds=false; //LOADS REGISTERED; ACTIVE LATER


//STRUCTS:

//logic: templates - class:

struct bbox {
    float x;
    float y;
    float z;

    float qw;
    float qx;
    float qy;
    float qz;

    float sx;
    float sy;
    float sz;

};

struct bbox_eigen {
    Eigen::Quaternionf quat;
    Eigen::Vector3f trans;
    Eigen::Vector3f scales;
    void toMM(){
        trans=trans*1000;
        scales=scales*1000;
    }
};

struct grasp {
    Eigen::Quaternionf quat;
    Eigen::Vector3f trans;
    std::vector<double> joints;
    grasp(){
        joints.resize(7); //for SDH
    }
};

struct kinesthetic {

    std::vector<double> joints;
    kinesthetic(){
        joints.resize(7); //for SDH
    }

};

//PROTOTYPES OF FUNCTIONS:

bbox_eigen convertBboxToBboxEigen(const bbox &  bbox_normal );

//functions
namespace fs = boost::filesystem;

void copyFile(const std::string & source, const std::string & target){

    fs::path to_fp(target);
    fs::path from_fp(source);

    if (fs::exists (to_fp))
        fs::remove (to_fp);

    fs::copy_file (from_fp, to_fp, fs::copy_option::overwrite_if_exists);
}

void createDir(const std::string & objectDir){

    boost::filesystem::path dir(objectDir);
    if(!boost::filesystem::exists(dir)){
      if (boost::filesystem::create_directory(dir))
        std::cout << "Directory creation:" << objectDir << " - Success" << "\n";
      else
      {
         cerr << "[Error!] Could not create dir path" << endl;
         exit(EXIT_FAILURE);
      }
    }

}

int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);

    //remove files which start with .

    vector<string> files2;
    for(int i=0; i < files.size(); i++){
        string xx(".");
        if(files.at(i).substr(0,1) != xx){
            files2.push_back(files.at(i));
        }
    }
    files=files2;

    return 0;
}

string intToStr(int nr){

    stringstream ss;
    ss << nr;
    return ss.str();
}

Eigen::Matrix4f mat4FromQuat(const Eigen::Quaternionf & quat,const Eigen::Vector3f trans){

    Eigen::Matrix4f mat4=Eigen::Matrix4f::Identity();
    mat4.block<3,3>(0,0)=quat.toRotationMatrix();
    mat4.col(3) << trans,1;

    return mat4;
}

template <typename T>
void remove_duplicates(std::vector<T>& vec)
{
  std::sort(vec.begin(), vec.end());
  vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPcInMM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_back=*cloud;
    for(int i=0; i < cloud_back->points.size(); i++){

        cloud_back->points.at(i).x *=1000;
        cloud_back->points.at(i).y *=1000;
        cloud_back->points.at(i).z *=1000;
    }

    return cloud_back;

}

void bboxesToMM(vector<bbox_eigen> & bboxes){

    for(int i=0; i < bboxes.size(); i++){
        bboxes.at(i).toMM();
    }
}

void cvMatdToEigenAffinef(const cv::Matx44d & cvMat, Eigen::Affine3f & eigeAffine){
   cv::Mat cvTr(4,4,CV_64FC1);
   cvTr=cv::Mat(cvMat);

   //Eigen::Map<Eigen::Matrix4d> eigenT( cvTr.data() );
   Eigen::Matrix4d converted;
   cv::cv2eigen(cvTr, converted );
   Eigen::Matrix4f eigen4f = converted.cast<float>();
   //convert to meters!

   eigen4f(0,3) *= 0.001;
   eigen4f(1,3) *= 0.001;
   eigen4f(2,3) *= 0.001;

   //cout << eigen4f << endl;
   Eigen::Affine3f x(eigen4f);
   eigeAffine=x;
}

cv::Matx44d eigenToCvMat(const Eigen::Quaterniond & quat, const Eigen::Vector3d & trans){

    Eigen::Matrix3d m = quat.toRotationMatrix();
    return cv::Matx44d( m(0,0), m(0,1), m(0,2), trans(0)*1000,
             m(1,0), m(1,1), m(1,2), trans(1)*1000,
             m(2,0), m(2,1), m(2,2), trans(2)*1000,
             0.0, 0.0, 0.0, 1.0 ); //to mm
}

//this thing is generating the prototype folder!

//TO BE IMPLEMENTED!!
bbox_eigen transformBB(const bbox_eigen & boundingbox, const Eigen::Matrix4f & trafo){

    bbox_eigen  boundingbox_transformed;
    Eigen::Matrix4f bbox_pose, bbox_pose_transformed;

    bbox_pose = mat4FromQuat(boundingbox.quat, boundingbox.trans);
    bbox_pose_transformed = trafo * bbox_pose;

    boundingbox_transformed.quat = Eigen::Quaternionf(bbox_pose_transformed.block<3,3>(0,0));
    boundingbox_transformed.trans = bbox_pose_transformed.col(3).head(3);
    boundingbox_transformed.scales = boundingbox.scales;

    return boundingbox_transformed;
}

void downSamplePointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, float leafSize){


        std::cout << "Downsampling Cloud with VoxelGrid: " << std::endl;
        std::cout << "Number of points before filtering " << pointcloud->size() << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::VoxelGrid< pcl::PointXYZRGB > sor;
        sor.setInputCloud (pointcloud);
        sor.setLeafSize (leafSize, leafSize, leafSize);
        sor.filter (*filteredCloud);
        pointcloud = filteredCloud;
        std::cout << "Number of points after filtering: " << pointcloud->size() << std::endl;


}

void passthroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, float min, float max, std::string xyz){

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (pointcloud);
  pass.setFilterFieldName (xyz.c_str());
  pass.setFilterLimits (min, max);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*pointcloud);




}

void cropPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, const bbox_eigen & bounding_box){

      Eigen::Matrix4f trafo = mat4FromQuat(bounding_box.quat,bounding_box.trans);

      pcl::transformPointCloud ((*pointcloud), (*pointcloud), trafo.inverse().eval() );

      float dx(bounding_box.scales(0));
      float dy(bounding_box.scales(1));
      float dz(bounding_box.scales(2));

      passthroughFilter(pointcloud, -dx/2,dx/2, "x");
      passthroughFilter(pointcloud, -dy/2,dy/2, "y");
      passthroughFilter(pointcloud, -dz/2,dz/2, "z");

      pcl::transformPointCloud ((*pointcloud), (*pointcloud), trafo);

      std::cout << "Number of points after cropping: " << pointcloud->size() << std::endl;

}

//factor has to be between 0 and 1
std::vector<double> interPolateJoints(const vector<double> & target,const  vector<double> & open, double factor){


    if(target.size()!=7 && open.size()!=7){
        cerr << "ERROR INTERPOLATE JOINTS" << endl;
        exit(EXIT_FAILURE);
    }

    vector<double> interpolated;
    interpolated.resize(7);
    for(int i=0; i < target.size(); i ++){
        interpolated[i] = factor*target[i] + (1-factor)*open[i];
    }
    return interpolated;

}

void loadClusterIds(const string & filename, vector<int> & clusterIds){

    clusterIds.clear();
    ifstream is(filename.c_str());
    string str;
    int line_nr=0;
    while(getline(is, str))
    {
        std::istringstream iss(str);
        if(line_nr==0){
            int value;
            while(iss >> value){
                clusterIds.push_back(value);
            }
        }
        else if(line_nr==1){
            break;
        }

        line_nr++;
    }
}

void computeOBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, bbox_eigen & bounding_box){


    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    float angleStep = 60.0f;
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.setInputCloud (pointcloud);
    feature_extractor.setAngleStep(angleStep);
    feature_extractor.compute ();

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    float width=max_point_OBB.x - min_point_OBB.x; //x-axis
    float height=max_point_OBB.y - min_point_OBB.y; //y-axis
    float depth=max_point_OBB.z - min_point_OBB.z;//z-axis

    bounding_box.trans(0)=position_OBB.x;
    bounding_box.trans(1)=position_OBB.y;
    bounding_box.trans(2)=position_OBB.z;

    bounding_box.quat = Eigen::Quaternionf(rotational_matrix_OBB);

    bounding_box.scales(0) = width;
    bounding_box.scales(1) = height;
    bounding_box.scales(2) = depth;

}

bool checkCollissionPointOBB(const pcl::PointXYZRGB & point, const bbox_eigen & boundingbox, const float & KernelSizePoint){

    Sphere3<float> sphere;
    Vector3<float> center;

    center.X()=point.x;
    center.Y()=point.y;
    center.Z()=point.z;

    sphere.Center = center;
    sphere.Radius = KernelSizePoint;
    //cout << "Collission Sphere Radius" << sphere.Radius << endl;

    Box3<float> box1;

    Eigen::Matrix4f trafo_bb1=mat4FromQuat(boundingbox.quat, boundingbox.trans);

    Vector3<float> bbox1_x_axis, bbox1_y_axis,bbox1_z_axis;
    Vector3<float> bbox1_center;

    bbox1_x_axis.X()=trafo_bb1(0,0);
    bbox1_x_axis.Y()=trafo_bb1(1,0);
    bbox1_x_axis.Z()=trafo_bb1(2,0);

    bbox1_y_axis.X()=trafo_bb1(0,1);
    bbox1_y_axis.Y()=trafo_bb1(1,1);
    bbox1_y_axis.Z()=trafo_bb1(2,1);

    bbox1_z_axis.X()=trafo_bb1(0,2);
    bbox1_z_axis.Y()=trafo_bb1(1,2);
    bbox1_z_axis.Z()=trafo_bb1(2,2);

    bbox1_center.X()=trafo_bb1(0,3);
    bbox1_center.Y()=trafo_bb1(1,3);
    bbox1_center.Z()=trafo_bb1(2,3);

    box1.Axis[0]=bbox1_x_axis;
    box1.Axis[1]=bbox1_y_axis;
    box1.Axis[2]=bbox1_z_axis;
    box1.Center=bbox1_center;

    box1.Extent[0]=boundingbox.scales(0)/2;
    box1.Extent[1]=boundingbox.scales(1)/2;
    box1.Extent[2]=boundingbox.scales(2)/2;

    IntrBox3Sphere3<float> intersectorSphereBox(box1,sphere);
    return intersectorSphereBox.Test();
}

bool checkCollissionPointOBB(const pcl::PointXYZRGB & point, const bbox_eigen & boundingbox){

    float KernelSizePoint = 0.01; //1 cm Kernel Size
    return checkCollissionPointOBB(point,  boundingbox, KernelSizePoint);


}

bool checkCollissionBB(const bbox_eigen & bounding_box1, const bbox_eigen & bounding_box2){

    Box3<float> box1;
    Box3<float> box2;

    Eigen::Matrix4f trafo_bb1=mat4FromQuat(bounding_box1.quat, bounding_box1.trans);
    Eigen::Matrix4f trafo_bb2=mat4FromQuat(bounding_box2.quat, bounding_box2.trans);

    Vector3<float> bbox1_x_axis, bbox1_y_axis,bbox1_z_axis;
    Vector3<float> bbox2_x_axis, bbox2_y_axis,bbox2_z_axis;
    Vector3<float> bbox1_center, bbox2_center;

    bbox1_x_axis.X()=trafo_bb1(0,0);
    bbox1_x_axis.Y()=trafo_bb1(1,0);
    bbox1_x_axis.Z()=trafo_bb1(2,0);

    bbox1_y_axis.X()=trafo_bb1(0,1);
    bbox1_y_axis.Y()=trafo_bb1(1,1);
    bbox1_y_axis.Z()=trafo_bb1(2,1);

    bbox1_z_axis.X()=trafo_bb1(0,2);
    bbox1_z_axis.Y()=trafo_bb1(1,2);
    bbox1_z_axis.Z()=trafo_bb1(2,2);

    bbox2_x_axis.X()=trafo_bb2(0,0);
    bbox2_x_axis.Y()=trafo_bb2(1,0);
    bbox2_x_axis.Z()=trafo_bb2(2,0);

    bbox2_y_axis.X()=trafo_bb2(0,1);
    bbox2_y_axis.Y()=trafo_bb2(1,1);
    bbox2_y_axis.Z()=trafo_bb2(2,1);

    bbox2_z_axis.X()=trafo_bb2(0,2);
    bbox2_z_axis.Y()=trafo_bb2(1,2);
    bbox2_z_axis.Z()=trafo_bb2(2,2);
    //////////
    bbox1_center.X()=trafo_bb1(0,3);
    bbox1_center.Y()=trafo_bb1(1,3);
    bbox1_center.Z()=trafo_bb1(2,3);

    bbox2_center.X()=trafo_bb2(0,3);
    bbox2_center.Y()=trafo_bb2(1,3);
    bbox2_center.Z()=trafo_bb2(2,3);

    box1.Axis[0]=bbox1_x_axis;
    box1.Axis[1]=bbox1_y_axis;
    box1.Axis[2]=bbox1_z_axis;

    box2.Axis[0]=bbox2_x_axis;
    box2.Axis[1]=bbox2_y_axis;
    box2.Axis[2]=bbox2_z_axis;

    box1.Center=bbox1_center;
    box2.Center=bbox2_center;

    box1.Extent[0]=bounding_box1.scales(0)/2;
    box1.Extent[1]=bounding_box1.scales(1)/2;
    box1.Extent[2]=bounding_box1.scales(2)/2;

    box2.Extent[0]=bounding_box2.scales(0)/2;
    box2.Extent[1]=bounding_box2.scales(1)/2;
    box2.Extent[2]=bounding_box2.scales(2)/2;

    IntrBox3Box3<float> intersectorBoxes(box1,box2);
    return intersectorBoxes.Test();
}

bool checkCollissionPointcloudOBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, const bbox_eigen & boundingbox, const float & kernelSize){

    //cout << "checking collission!" << endl;
    //cout << "pointcloud->points.size() " << pointcloud->points.size() << endl;

    for(int i=0; i<pointcloud->points.size(); i++){

        if(checkCollissionPointOBB(pointcloud->points.at(i), boundingbox, kernelSize)){
            //cout << pointcloud->points.at(i).x << endl;
            //cout << "!!!COLLISSION TRUE!!!" << endl;
            return true;
        }
    }

    //cout << "!!!COLLISSION FALSE!!!" << endl;

    return false;

}

bool checkCollissionPointcloudOBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, const bbox_eigen & boundingbox){

    cout << "checking collission!" << endl;
    cout << "pointcloud->points.size() " << pointcloud->points.size() << endl;

    for(int i=0; i<pointcloud->points.size(); i++){

        if(checkCollissionPointOBB(pointcloud->points.at(i), boundingbox)){
            //cout << pointcloud->points.at(i).x << endl;
            cout << "!!!COLLISSION TRUE!!!" << endl;
            return true;
        }
    }

    cout << "!!!COLLISSION FALSE!!!" << endl;

    return false;

}

void generateBBoxesFromJoints(vector<double> & joints, vector<bbox_eigen> & allBoxesEigen){

   vector <bbox> allBboxes;
   //cout << "Joints:" << endl;
   //cout << joints.size() << endl;
   //for(int i=0; i < joints.size(); i++){
   //    cout << joints.at(i) << endl;
   //}

    Eigen::Quaterniond offset_left_arm_quat;
    Eigen::Vector3d   offset_left_arm_trans;

    offset_left_arm_quat.w()=0.70711;
    offset_left_arm_quat.x()=0;
    offset_left_arm_quat.y()=0;
    offset_left_arm_quat.z()=-0.70711;
    offset_left_arm_trans << 0,0,0.0586;

    ForwardKinematics forwardKinematics;
    forwardKinematics.setAngles(joints); //For now use left arm as used for database

    forwardKinematics.setGlobalTransformation(eigenToCvMat(offset_left_arm_quat, offset_left_arm_trans));

    std::vector< cv::Matx44d > allFrames_center;

    float lx=0.098;
    float sx=0.12;

    bbox palm_link;
    palm_link.x=0;
    palm_link.y=0;
    palm_link.z=0.0586;
    palm_link.qw=0.70711;
    palm_link.qx=0;
    palm_link.qy=0;
    palm_link.qz=-0.70711;
    palm_link.sx=sx;
    palm_link.sy=sx;
    palm_link.sz=lx;

    allBboxes.push_back(palm_link);

    for(int m=0; m < 7; m++){
    allFrames_center.push_back(forwardKinematics.getLinkCenters(m));
    }
    for(int m=0; m < 6; m++){

        Eigen::Affine3f current_frame_center;
        cvMatdToEigenAffinef(allFrames_center.at(m), current_frame_center);

        bbox current_link;
        //draw boxes :)
        float l1(0.0865), l2(0.0685),s(0.03);

        if(m==0 || m==2 || m==4){

            Eigen::Quaternionf quat(current_frame_center.rotation());
            Eigen::Vector3f trans=current_frame_center.translation();

            current_link.x=trans(0);
            current_link.y=trans(1);
            current_link.z=trans(2);
            current_link.qw=quat.w();
            current_link.qx=quat.x();
            current_link.qy=quat.y();
            current_link.qz=quat.z();
            current_link.sx=l1;
            current_link.sy=s;
            current_link.sz=s;

        }
        else{
            Eigen::Quaternionf quat(current_frame_center.rotation());
            Eigen::Vector3f trans=current_frame_center.translation();

            current_link.x=trans(0);
            current_link.y=trans(1);
            current_link.z=trans(2);
            current_link.qw=quat.w();
            current_link.qx=quat.x();
            current_link.qy=quat.y();
            current_link.qz=quat.z();
            current_link.sx=l2;
            current_link.sy=s;
            current_link.sz=s;

        }

        allBboxes.push_back(current_link);
    }

    //convert all bboxes to eigen bboxes

    allBoxesEigen.clear();

    for(int i = 0 ; i < allBboxes.size(); i++){

        bbox_eigen curr;
        curr = convertBboxToBboxEigen(allBboxes.at(i));
        allBoxesEigen.push_back(curr);
    }

    //cout << "[debug] size allboxes:" << endl;
    //cout << allBoxesEigen.size() << endl;

}

std::string removeAbsPath(const std::string& filename) {
    size_t lastslash = filename.find_last_of("/");
    size_t last=filename.size() -1;
    //if (lastslash == std::string::npos) return filename;
    return filename.substr(lastslash+1, last);
}

vector<double> interPolateJointsUntilCollissionFree(const vector<double> & target,
                                                    const  vector<double> & open,
                                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud){

    //make kernel size large enough such that distance is about 2 cm, which makes kernelsize ~ 4 cm
    //l

    for(float factor=1; factor > 0; factor = factor - 0.01 ){
        vector<double> currentJoints;
        vector<bbox_eigen> sdh_bbox;
        currentJoints=interPolateJoints(target, open,factor);
        generateBBoxesFromJoints(currentJoints,sdh_bbox);

        bool collides = true;
        for( int link=0; link < sdh_bbox.size(); link++){
            collides=true;

            if(checkCollissionPointcloudOBB(pointcloud,sdh_bbox.at(link), 0.02f)){
                //cout << "COLLISSION DETECTED LINK:" << link << endl;
                collides=true;
                break;
            }
            collides=false; //means none of them collides, means no break and setting false after last exit

        }

        if(!collides){
            cout << "COLLISSION FACTOR:" << factor << endl;

            return currentJoints;
        }



    }

    cerr << "NO COLLISSION FREE INTERPOLATION POSSIBLE!!! RETURNING PREGRASP" << endl;
    return open;


}

bbox_eigen convertBboxToBboxEigen(const bbox &  bbox_normal ){

    bbox_eigen bbox_condensed;
    bbox_condensed.trans << bbox_normal.x, bbox_normal.y, bbox_normal.z;
    bbox_condensed.quat = Eigen::Quaternionf(bbox_normal.qw, bbox_normal.qx, bbox_normal.qy,  bbox_normal.qz);
    bbox_condensed.scales << bbox_normal.sx, bbox_normal.sy, bbox_normal.sz;


    return bbox_condensed;

}

//computes in hand frame only :)

class grasp_template{

public:

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_template_gen; //use color or color yourself
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_template_registered; //use by registered data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_template_gen_ori;

    vector<bbox_eigen> roi_gripper; //for gripper
    bbox_eigen roi_template; //BBOX for grasp itself, same as the one for generation :)
    //string preshape_template; not needed as copied by aquisition! xml files are subsequently generated by
    bbox_eigen roi_template_ori;
    string original_folder_path; //to copy tactile and kinesthetics.txt
    string generated_folder_path; //where to write the stuff

    grasp grasp_data;
    grasp pre_grasp;
    grasp post_grasp;
    kinesthetic grasp_joints;
    vector<double> gen_pre_grasp_joints;
    //kinesthetic gen_pre_grasp_joints;

    //MAYBE SAVE JOINTS TO BE LOADED DIRECTLY BY PBP PLANNER FOR COLISSION CHECKING IN MOVEIT :)
    //////////////////INTERMEDIATE_THINGS/////// -> LOAD THEN DIRECT BY EXECUTOR AS ONLY RELATIVE POSE HAS TO BE USED, GOOD TO BE LOADED DIRECTLY :)

    grasp loadGrasp(const string filepath){

        grasp lg;

        ifstream is(filepath.c_str());
        string str;
        int line_nr=0;
        while(getline(is, str))
        {
            std::istringstream iss(str);
            if(line_nr==0){
                iss >> lg.trans(0) >> lg.trans(1) >> lg.trans(2);
                iss >> lg.quat.w() >> lg.quat.x() >> lg.quat.y() >> lg.quat.z();
            }
            else if(line_nr==1){
                for(int i=0 ; i < 7; i++){
                    iss >> lg.joints.at(i);
                }
                break;
            }
            line_nr++;
        }

        return lg;
    }

    kinesthetic loadKinesthetic(const string filepath){
        kinesthetic lg;

        ifstream is(filepath.c_str());
        string str;
        int line_nr=0;
        while(getline(is, str))
        {
            std::istringstream iss(str);
            if(line_nr==0){
                 for(int i=0 ; i < 7; i++){
                    iss >> lg.joints.at(i);
                    //cout << iss.str() << endl;
                }
                break;
            }
            line_nr++;
        }
        //cout << "[debug] grasp joints after loading:" << endl;
        //cout << lg.joints.at(0) << endl;
        return lg;
    }

    int getNrPoints(){

        return grasp_template_gen->points.size();
    }

    void loadAllGrasps(const string & pathDir){
        string pre_grasp_str("/pre_grasp.txt"),grasp_str("/grasp.txt"), post_grasp_str("/post_grasp.txt"), kinesthetic_str("/kinesthetic.txt");

        pre_grasp = loadGrasp(pathDir + pre_grasp_str);
        grasp_data = loadGrasp(pathDir + grasp_str);
        post_grasp = loadGrasp(pathDir + post_grasp_str);
        grasp_joints = loadKinesthetic(pathDir + kinesthetic_str);

    }
    //to be calles after writeROI GRIPPER!!
    void savePreGraspJointsgen(const string & filename){

        ofstream of;
        of.open(filename.c_str());
        if(of.is_open()){
            for(int i = 0; i < gen_pre_grasp_joints.size(); i++){
                of << gen_pre_grasp_joints.at(i) << " ";
            }
        }
        of.close();

    }

    void writeROIGripper(const string & filename){

        //for(float factor=0; factor < 1; factor = factor + 0.1){


        roi_gripper.clear();
        vector<double> joints_interpolated=interPolateJointsUntilCollissionFree( grasp_joints.joints, pre_grasp.joints, grasp_template_gen);
        gen_pre_grasp_joints = joints_interpolated;
        //vector<double> joints_interpolated=interPolateJoints( grasp_joints.joints, pre_grasp.joints, factor);
        generateBBoxesFromJoints(joints_interpolated , roi_gripper); //use kinesthetic instead of grasp


        //to MM

        //bboxesToMM(roi_gripper);
          ofstream of;
          of.open(filename.c_str());
          if(of.is_open()){
              for(int i = 0; i < roi_gripper.size(); i++){
                  //cout << "[debug] in gripper loop" << endl;
                  bbox_eigen bb=roi_gripper.at(i);
                    bb.toMM();
                  of << bb.trans(0) << " " <<  bb.trans(1) << " " << bb.trans(2) << " ";
                  of << bb.quat.w() << " " << bb.quat.x() << " " <<  bb.quat.y() << " " << bb.quat.z() << " ";
                  of << bb.scales(0) << " " <<  bb.scales(1) << " " << bb.scales(2) << "\n";
              }

          }
          of.close();
        //}


    }

    void writeTestROIGripper(const string & filename){

        for(float factor=0; factor < 1; factor = factor + 0.1){


        roi_gripper.clear();
        //vector<double> joints_interpolated=interPolateJointsUntilCollissionFree( grasp_joints.joints, pre_grasp.joints, grasp_template_gen);

        vector<double> joints_interpolated=interPolateJoints( grasp_joints.joints, pre_grasp.joints, factor);
        generateBBoxesFromJoints(joints_interpolated , roi_gripper); //use kinesthetic instead of grasp

        //check collission

        vector<double> currentJoints=joints_interpolated;
        vector<bbox_eigen> sdh_bbox;
        generateBBoxesFromJoints(currentJoints,sdh_bbox);
        bool collides = true;
        for( int link=0; link < sdh_bbox.size(); link++){
            collides=true;

            if(checkCollissionPointcloudOBB(grasp_template_gen,sdh_bbox.at(link), 0.0f)){
                //cout << "COLLISSION DETECTED LINK:" << link << endl;
                collides=true;
                break;
            }
            collides=false; //means none of them collides, means no break and setting false after last exit

        }


        cout << factor << "collides:" << collides << endl;
        //bboxesToMM(roi_gripper);
          ofstream of;
          stringstream fn;
          fn << filename << factor << ".roi";
          of.open(fn.str().c_str());
          if(of.is_open()){
              for(int i = 0; i < roi_gripper.size(); i++){
                  //cout << "[debug] in gripper loop" << endl;
                  bbox_eigen bb=roi_gripper.at(i);
                    bb.toMM();
                  of << bb.trans(0) << " " <<  bb.trans(1) << " " << bb.trans(2) << " ";
                  of << bb.quat.w() << " " << bb.quat.x() << " " <<  bb.quat.y() << " " << bb.quat.z() << " ";
                  of << bb.scales(0) << " " <<  bb.scales(1) << " " << bb.scales(2) << "\n";
              }

          }
          of.close();
        }


    }

    void writeROITemplate(const string & filename){



          ofstream of;
          of.open(filename.c_str());
          if(of.is_open()){
                  bbox_eigen bb=roi_template;

                  bb.toMM();

                  of << bb.trans(0) << " " <<  bb.trans(1) << " " << bb.trans(2) << " ";
                  of << bb.quat.w() << " " << bb.quat.x() << " " <<  bb.quat.y() << " " << bb.quat.z() << " ";
                  of << bb.scales(0) << " " <<  bb.scales(1) << " " << bb.scales(2) << "\n";
          }
          of.close();


    }

    void generateTemplate(const string & folderPath, bbox_eigen roi, const string &  gen_folder_path){


        Eigen::Matrix4f trafo = mat4FromQuat(grasp_data.quat,grasp_data.trans); //actually this would be loaded from the grasp pose!
        //invert with this matrix!
        //roi_template = roi;
        original_folder_path = folderPath;
        generated_folder_path = gen_folder_path; //going one level deeper I hope

        //load pointcloud;

        if(use_registered_clouds==true){

            pcl::io::loadPCDFile(folderPath + "/cloud_registered.pcd", *grasp_template_registered);

            pcl::transformPointCloud ((*grasp_template_registered), (*grasp_template_registered), trafo.inverse().eval() );
            downSamplePointcloud(grasp_template_registered,0.02); //voxelsize, maybe this needs to be done earlier??
            cropPointcloud(grasp_template_registered,roi);

        }
        else{

           pcl::io::loadPCDFile(folderPath + "/cloud_segmented.pcd", *grasp_template_gen);



           pcl::transformPointCloud ((*grasp_template_gen), (*grasp_template_gen), trafo.inverse().eval() );
           downSamplePointcloud(grasp_template_gen, 0.005); //voxelsize, maybe this needs to be done earlier??

            cout << "Pointcloudsize: " << grasp_template_gen->points.size() << endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            *cloud_tmp=*grasp_template_gen;

            //downSamplePointcloud(cloud_tmp,0.03);

            if(cloud_tmp->points.size()!=0){

               cout << "Writing transformed cloud" << endl;

           pcl::io::savePCDFile(folderPath + "/cloud_transformed.pcd", *cloud_tmp);
           }




           cropPointcloud(grasp_template_gen,roi);
           computeOBB(grasp_template_gen,roi_template); //makes reall bbox after cutting

           //transform back for collission checking
           roi_template_ori = transformBB(roi_template,trafo);
           pcl::transformPointCloud((*grasp_template_gen), (*grasp_template_gen_ori), trafo);


           if(grasp_template_gen->points.size()!=0){
               cout << "Writing cropped cloud" << endl;
           pcl::io::savePCDFile(folderPath + "/template_gen.pcd", *getPcInMM(grasp_template_gen));
           }

           if(grasp_template_gen_ori->points.size()!=0){
               cout << "Writing template original frame" << endl;
           pcl::io::savePCDFile(folderPath + "/template_gen_ori.pcd", *grasp_template_gen_ori);
           }

        }

        //Now generate the hand bounding boxes //can in principle also generated inside nuklei, doesnt matter
    }

    void saveTemplates(const string & targetPrefix){

        //target prefix may be path/0 .. 1 -> 1.tactile, 0.pcd, 0.kinesthetics ...

        //first copy pre_existing stuff:
        copyFile(original_folder_path + "/tactile.txt" , targetPrefix+".tac");
        copyFile(original_folder_path + "/kinesthetic.txt" , targetPrefix+".kin");
        copyFile(original_folder_path + "/pre_grasp.txt" , targetPrefix+".pre");
        copyFile(original_folder_path + "/post_grasp.txt" , targetPrefix+".post");
        copyFile(original_folder_path + "/grasp.txt" , targetPrefix+".grasp");

        //generated stuff:
        writeROIGripper(targetPrefix+".roi");
        //write collision pre joints gen
        savePreGraspJointsgen(targetPrefix+".pregen");
        //bbox of template
        writeROITemplate(targetPrefix+".bbox");
        //pointcloud of template
        if(grasp_template_gen->points.size()!=0){
            cout << "Writing cloud" << endl;
        pcl::io::savePCDFile(targetPrefix+".pcd", *getPcInMM(grasp_template_gen));
        }

        stringstream nuklei_cmd;
        nuklei_cmd << "nuklei conv -w nuklei " << targetPrefix <<".pcd " << targetPrefix+".xml";

        stringstream nuklei_cmd_ply;
        nuklei_cmd_ply << "nuklei conv -w ply " << targetPrefix <<".pcd " << targetPrefix+".ply";

          int i;

          if (system(NULL)) puts ("Ok");
            else exit (EXIT_FAILURE);
          cout << "Executing command " << nuklei_cmd.str() << endl;
          i=system (nuklei_cmd.str().c_str());
          printf ("The value returned was: %d.\n",i);

          int j;

          if (system(NULL)) puts ("Ok");
            else exit (EXIT_FAILURE);
          cout << "Executing command " << nuklei_cmd_ply.str() << endl;
          j=system (nuklei_cmd_ply.str().c_str());
          printf ("The value returned was: %d.\n",i);



        //here system call to generate .xml file by nuklei

          //write preshape
          string dirPath=targetPrefix+".preshape";
          ofstream of(dirPath.c_str());
          if(of.is_open()){

             of << "pinch" << endl; //for now just all pinch grasps
          }
          of.close();


    }


    //constructor
    grasp_template() {

        grasp_template_gen.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        grasp_template_gen_ori.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        grasp_template_registered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

};

//tacile muss derzeit nicht verarbeitet werden!
//all data is using rgb pointclouds?? every other is

grasp_template selectTemplateBasedOnName(const vector<grasp_template> & templateList, const string & name_wo_prefix ){


    for(int i=0; i < templateList.size(); i++){

        grasp_template current_template = templateList.at(i);
        //cout << "Comparing Name of grasp_obj modified: " << endl;
        string grasp_folder_mod = removeAbsPath(current_template.original_folder_path);

        if(grasp_folder_mod == name_wo_prefix ) {
            return current_template;
        }
    }

    cerr << "NO MATCH!, RETURNING EMPTY TEMPLATE" << endl;

    return grasp_template();

}

float distancePartTemplate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, const bbox_eigen & boundingbox){

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pointcloud,centroid);
    Eigen::Vector3f dist = centroid.head(3) - boundingbox.trans;
    return dist.norm();
}

struct associationHypothesis{
    int cluster_id;
    string grasp_name;
};

struct sortHyp {
    bool operator()(const std::pair<float,associationHypothesis> &left, const std::pair<float,associationHypothesis> &right) {
        return left.first < right.first;
    }
};



int main(int argc, char** argv){

    //HARDCODED: ROI_BBOX FOR GRASP TEMPLATE defined in gripper frame,
    //can in future also depend upon the grasp joints also, if 2 or more tactile contacts are registered - could use 3D hull or so
    //Algorithm -> for every part - cluster a number of templates are generated

    /*for every object_id
        for every part_of_object (folder 0 ... n)  if cluster_id
            generate map cluster_id -> string_grasp folder
      endfor
      */

    int NR_PARTS=0;
    int NR_COLLISSIONS=0;
    int NR_Clusters = 50; //can be higher as real amount

    vector< vector<string> > grasp_of_cluster; //maybe better to load all of them in the beginning?
    grasp_of_cluster.resize(NR_Clusters);

    // eg. grasp_of_cluster[0] are all grasps of cluster 0, by string - if multiple are used check and delete - list of templates can
    //be generated anyhow:, this is just used for assignment

    //test one of them

    bbox_eigen roi_all_templates;

    roi_all_templates.trans << 0,0,0.22;
    roi_all_templates.quat=Eigen::Quaternionf(1,0,0,0);
    roi_all_templates.scales << 0.1,0.1,0.10; //To be tested - very important

    //loop over templates
    string dirParts = string("../pacman_pilot/parts-recog-all");
    string dirGrasps = string("../pacman_pilot/grasp_db");
    string dirTemplatesGen= "../pacman_pilot/generated_templates";

    vector<string> filesObjects = vector<string>();
    vector<string> filesGrasps = vector<string>();

    getdir(dirParts,filesObjects);
    getdir(dirGrasps,filesGrasps);

    cout << "Objects in Grasp Folder:" << endl;
    for (unsigned int i = 0;i < filesGrasps.size();i++) {
    cout << filesGrasps[i] << endl;
    }
    cout << "Objects in Part Folder:" << endl;
    for (unsigned int i = 0;i < filesObjects.size();i++) {
    cout << filesObjects[i] << endl;
    }

    ///CREATE LIST OF GRASP TEMPLATES
    ///vector of templates - does only make sense for single object and grasp per folder!

    //IF POINTS EXCEED CERTAIN NUMBER; WE WILL GENERATE THE TEMPLATE:
    cout << "Generating Templates for all object grasp combinations" << endl;
    vector<grasp_template> listTemplates;
    for(int i=0; i < filesGrasps.size(); i++){

        grasp_template graspTemplate;
        graspTemplate.loadAllGrasps(dirGrasps + "/" + filesGrasps[i]);
        graspTemplate.generateTemplate(dirGrasps + "/" + filesGrasps[i], roi_all_templates, dirTemplatesGen + "/" + filesGrasps[i]); //for now use 1 ROI and look what happens
        graspTemplate.writeROIGripper(dirGrasps + "/" + filesGrasps[i] + "/sdh.roi");
        graspTemplate.writeROITemplate(dirGrasps + "/" + filesGrasps[i] + "/template.roi");
        if(i==0){
            graspTemplate.writeTestROIGripper(dirGrasps + "/" + filesGrasps[i] + "/sdh");
        }
        cout <<graspTemplate.getNrPoints() << endl;
        if(graspTemplate.getNrPoints() > 100){
        listTemplates.push_back(graspTemplate);
        }

    }

    cout << "NR TEMPLATES GENERATED: " << listTemplates.size() << "/" << filesGrasps.size() << endl;
    ////MAIN LOOP///

    cout << "Main Loop: Looading recognized parts and associating grasp to clusters" << endl;
    for (int obj_id=0; obj_id < filesObjects.size(); obj_id++){

        vector<string> filesParts = vector<string>();
        cout << "Current Object " << dirParts + "/" + filesObjects.at(obj_id) << endl;
        getdir(dirParts + "/" + filesObjects.at(obj_id),filesParts); //get part folder of current object :)
        cout << "current parts:" << endl;
        for (unsigned int i = 0;i < filesParts.size();i++) {
        cout << filesParts[i] << endl;
        }

        ///LOOP over PARTS of current object

        //vector<string> grasp_names;
        vector < std::pair<float, associationHypothesis> > distHypothesis; //all hypothesis

        for(int part_id=0; part_id < filesParts.size(); part_id ++){

            NR_PARTS++;

            string pcdFile = dirParts + "/" + filesObjects.at(obj_id) + "/" + filesParts.at(part_id) + "/part.pcd";
            string idFile = dirParts + "/" + filesObjects.at(obj_id) + "/" + filesParts.at(part_id) + "/id.txt";

            bbox_eigen partBBox;
            vector<int> clusterIds;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPart(new pcl::PointCloud<pcl::PointXYZRGB>);
            loadClusterIds(idFile, clusterIds); //mostly single ids
            pcl::io::loadPCDFile(pcdFile, *currentPart);
            computeOBB(currentPart, partBBox);

            //Printing cluster ids

            cout << "Current Cluster Ids:" <<  endl;
            for(int i = 0; i< clusterIds.size();i++){
                cout << clusterIds.at(i) << " ";
            }
            cout <<  endl;
            //now check if bbox collides with list of templates of this special object!

            for(int i=0; i < listTemplates.size(); i++){

                /*
                  *Condition for association:
                  * If for current recognized Part:
                    1. Grasp is performed on same object
                    2. Bounding box of grasp_template and part actually do collide //maybe another assumtion is better later, convex hull?
                    3. No multiple associations, per object_grasp - for all parts who touch only one grasp should be associated, so push back a list and select the one which is closest

                    -> then associate cluster with current template: output : /clusterId/0..n templates for current cluster
                  */
                //first check if object is the same!
                grasp_template current_template = listTemplates.at(i);
                //cout << "Comparing Name of grasp_obj modified: " << endl;
                string grasp_folder_mod = removeAbsPath(current_template.original_folder_path) + "_segmented";



                //cout << removeAbsPath(current_template.original_folder_path) + "_segmented" << endl;
                //cout << "with " <<endl;
                //cout << filesObjects.at(obj_id) <<endl;
                //1
                if(grasp_folder_mod != filesObjects.at(obj_id) ) continue;
                //2
                //if(!checkCollissionBB(current_template.roi_template_ori, partBBox)) continue;
                if(!checkCollissionPointcloudOBB(currentPart,current_template.roi_template_ori)) continue;
                NR_COLLISSIONS++;
                //measure distance of bounding box to part

                float currentDistancePartTemplate = distancePartTemplate(currentPart, current_template.roi_template_ori);
                cout << "\n----ASSOCIATION----\n";
                string grasp_name = removeAbsPath(current_template.original_folder_path);
                std::pair<float, associationHypothesis> currentHyp;



                for(int j=0; j < clusterIds.size(); j++){
                    //3
                    //collect here all hypothetical associations
                    currentHyp.first=currentDistancePartTemplate;
                    currentHyp.second.grasp_name=grasp_name;
                    currentHyp.second.cluster_id=clusterIds[j];
                    distHypothesis.push_back(currentHyp);
                    //grasp_of_cluster[clusterIds[j]].push_back( grasp_name ); //appending the std::vector - define append operator ;)
                }
            }
        }

        //sort the associations:

        if(distHypothesis.size()>0){
            std::sort(distHypothesis.begin(),distHypothesis.end(), sortHyp());
        associationHypothesis clostestHyp = distHypothesis.at(0).second;//clossest hypothesis
        grasp_of_cluster[clostestHyp.cluster_id].push_back(clostestHyp.grasp_name );
        }



    }

    //Test output:
    cout << "Active Clusters:" << endl;

    for (int i = 0; i < grasp_of_cluster.size(); i++){


        if (grasp_of_cluster.at(i).size() != 0){

            //cout << "removing duplicates" << endl;
            remove_duplicates(grasp_of_cluster.at(i));
            cout << i << "  " << grasp_of_cluster.at(i).size() << "  ";

            for(int j=0; j < grasp_of_cluster.at(i).size(); j++){
                cout << grasp_of_cluster.at(i).at(j) << " ";
            }
            cout << endl;

        }
    }
    cout << "Nr of collission/parts" << NR_COLLISSIONS << "/" << NR_PARTS << endl;

    //NOW GENERATING THE DIRECTORIES FOR TARGET:

    //dirTemplatesGen

    cout << "Creating Target Directories: " << dirTemplatesGen << endl;

    createDir(dirTemplatesGen);

    for (int i = 0; i < grasp_of_cluster.size(); i++){


            if (grasp_of_cluster.at(i).size() != 0){

                //Do stuff on cluster level!
                string currentDir=dirTemplatesGen + "/" + intToStr(i);
                string id_file=currentDir+"/ids.txt";
                createDir(currentDir);
                createDir(currentDir+"/prototypes");

                ofstream of(id_file.c_str());//writing ids

                //End do stuff

                cout << i << "  " << grasp_of_cluster.at(i).size() << "  ";

                for(int j=0; j < grasp_of_cluster.at(i).size(); j++){
                    cout << grasp_of_cluster.at(i).at(j) << " ";

                    //Do stuff inside cluster level i!

                    //write ids.txt file
                    if(of.is_open()){
                        of << j << " " << grasp_of_cluster.at(i).at(j) << endl;
                    }

                    //copy the original folder files and rename - make member function of template so to access also stuff inside

                    cout << "GENERATION FOR: " << currentDir+"/prototypes/" + intToStr(j) << endl;
                    grasp_template currTemplate = selectTemplateBasedOnName(listTemplates, grasp_of_cluster.at(i).at(j));
                    currTemplate.saveTemplates(currentDir+"/prototypes/" + intToStr(j) );
                    //End do stuff
                }
                of.close();
                //cout << endl;

            }
        }

  return 0;
}
