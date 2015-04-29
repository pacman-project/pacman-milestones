#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tclap/CmdLine.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <algorithm>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <Eigen/Dense>
//#include <opencv2/core/eigen.hpp>

//for system call to nuklei

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */

#include <iostream>
#include <iomanip>

//boost
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#define DEBUG_TEMPLATE 1

using namespace std;

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
//get Minor axis, should be much morestable and main axis eigenvalue=proportional to size of pointcloud
void getCentroidAndMainAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,
                            Eigen::Vector3f & centroid,
                            Eigen::Vector3f & minor_axis,float & eigenValueMain){
    Eigen::Vector3f middle, minor, main_ax;
    float m1,m2,m3;

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    float angleStep = 60.0f;
    feature_extractor.setInputCloud (pointcloud);
    feature_extractor.setAngleStep(angleStep);
    feature_extractor.compute ();

    feature_extractor.getMassCenter(centroid);
    feature_extractor.getEigenVectors(main_ax,middle,minor_axis);
    feature_extractor.getEigenValues(eigenValueMain,m2,m3);

}

//get Minor axis, should be much morestable and main axis eigenvalue=proportional to size of pointcloud
void getCentroidAndFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,
                            Eigen::Vector3f & centroid,
                            Eigen::Vector3f & minor_axis,
                            Eigen::Vector3f & middle_axis,
                            Eigen::Vector3f & major_axis,
                            float & eigenValueMinor,
                            float & eigenValueMiddle,
                            float & eigenValueMajor)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;

    float angleStep = 60.0f;
    feature_extractor.setInputCloud (pointcloud);
    feature_extractor.setAngleStep(angleStep);
    feature_extractor.compute();

    feature_extractor.getMassCenter(centroid);
    feature_extractor.getEigenVectors(major_axis,middle_axis,minor_axis);
    feature_extractor.getEigenValues(eigenValueMajor,eigenValueMiddle,eigenValueMinor);

}


void getFeatureVector(const string & partDir,
                      vector <vector<float> > & feature_vector)
{

    feature_vector.clear();

    string dirParts = partDir;

    vector<string> filesObjects = vector<string>();

    getdir(dirParts,filesObjects);

    cout << "Objects in Part Folder:" << endl;
    for (unsigned int i = 0;i < filesObjects.size();i++) {
        cout << filesObjects[i] << endl;
    }




    cout << "Main Loop: Looading recognized parts and compute features on them" << endl;
    for (int obj_id=0; obj_id < filesObjects.size(); obj_id++){

        vector<string> filesParts = vector<string>();
        cout << "Current Object " << dirParts + "/" + filesObjects.at(obj_id) << endl;
        getdir(dirParts + "/" + filesObjects.at(obj_id),filesParts); //get part folder of current object :)
        cout << "current parts:" << endl;
        for (unsigned int i = 0;i < filesParts.size();i++) {
            cout << filesParts[i] << endl;
        }

        ///LOOP over PARTS of current object
        //for(int part_id=0; part_id < filesParts.size(); part_id ++){


        //string pcdFile = dirParts + "/" + filesObjects.at(obj_id) + "/" + filesParts.at(part_id) + "/part.pcd";
        //string idFile = dirParts + "/" + filesObjects.at(obj_id) + "/" + filesParts.at(part_id) + "/id.txt";

        string pcdFile = dirParts + "/" + filesObjects.at(obj_id) + "/part.pcd";
        string idFile = dirParts + "/" + filesObjects.at(obj_id) + "/id.txt";


        vector<int> clusterIds;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPart(new pcl::PointCloud<pcl::PointXYZRGB>);
        loadClusterIds(idFile, clusterIds); //only best single ids
        pcl::io::loadPCDFile(pcdFile, *currentPart);

        Eigen::Vector3f centroid, major_axis, middle_axis, minor_axis ;
        vector<float> part_f_vector;
        float ev_major, ev_middle, ev_minor, nrPointsCloud;

        nrPointsCloud = currentPart->points.size();
        getCentroidAndFeatures(currentPart,
                               centroid,
                               minor_axis,
                               middle_axis,
                               major_axis,
                               ev_minor,
                               ev_middle,
                               ev_major
                               );

        ///////PRINTING STUFF:
        cout << "Current Cluster-Id:" <<  endl;
        for(int i = 0; i< clusterIds.size();i++){
            cout << clusterIds.at(i) << " ";
        }
        cout <<  endl;



        if(clusterIds.size()==0){

            cout << "Current Filename:" << endl;
            cout << filesObjects.at(obj_id) << endl;
            cout << "Current Centroid:" <<  endl;
            cout << centroid.transpose()*1000 << endl;
            cout << "Current Minor Axis:" <<  endl;
            cout << minor_axis.transpose() << " " << minor_axis.norm() << endl;


            part_f_vector.push_back(centroid(0));
            part_f_vector.push_back(centroid(1));
            part_f_vector.push_back(centroid(2));
            part_f_vector.push_back(minor_axis(0));
            part_f_vector.push_back(minor_axis(1));
            part_f_vector.push_back(minor_axis(2));
            part_f_vector.push_back(-1); //cluster_id
            part_f_vector.push_back(ev_major*1000);
            part_f_vector.push_back(middle_axis(0));
            part_f_vector.push_back(middle_axis(1));
            part_f_vector.push_back(middle_axis(2));
            part_f_vector.push_back(ev_middle*1000);
            part_f_vector.push_back(major_axis(0));
            part_f_vector.push_back(major_axis(1));
            part_f_vector.push_back(major_axis(2));
            part_f_vector.push_back(ev_minor*1000);
            part_f_vector.push_back(nrPointsCloud);
            part_f_vector.push_back(obj_id);

        }

        if(clusterIds.size()==1){

            cout << "Current Filename:" << endl;
            cout << filesObjects.at(obj_id) << endl;
            cout << "Current Centroid:" <<  endl;
            cout << centroid.transpose()*1000 << endl;
            cout << "Current Minor Axis:" <<  endl;
            cout << minor_axis.transpose() << " " << minor_axis.norm() << endl;

            part_f_vector.push_back(centroid(0));
            part_f_vector.push_back(centroid(1));
            part_f_vector.push_back(centroid(2));
            part_f_vector.push_back(minor_axis(0));
            part_f_vector.push_back(minor_axis(1));
            part_f_vector.push_back(minor_axis(2));
            part_f_vector.push_back(clusterIds[0]); //cluster_id
            part_f_vector.push_back(ev_major*1000);
            part_f_vector.push_back(middle_axis(0));
            part_f_vector.push_back(middle_axis(1));
            part_f_vector.push_back(middle_axis(2));
            part_f_vector.push_back(ev_middle*1000);
            part_f_vector.push_back(major_axis(0));
            part_f_vector.push_back(major_axis(1));
            part_f_vector.push_back(major_axis(2));
            part_f_vector.push_back(ev_minor*1000);
            part_f_vector.push_back(nrPointsCloud);
            part_f_vector.push_back(obj_id);

        }

        feature_vector.push_back(part_f_vector);
    }


    //}
}

//always 2 minus
Eigen::Matrix4f flipFrame(const Eigen::Matrix4f & frame, float x, float y, float z){

    Eigen::Matrix4f flippedFrame;
    flippedFrame = frame;

    flippedFrame.col(0)=x*frame.col(0);
    flippedFrame.col(1)=y*frame.col(1);
    flippedFrame.col(2)=z*frame.col(2);

    return flippedFrame;
}

struct alignmentHyp{

    Eigen::Matrix4f trafo;
    int part_id_obj;
    int part_id_scene;

};

float relativeSize(float x1, float x2){

    float x1mod=x1+0.0001;

    float relSz=fabs(x1-x2)/std::max(x1mod,x2);
    return relSz;

}

bool compareFVec(const vector<float> & f1, const vector<float> & f2, float threshold){

    //all values should be within threshold
    if(f1.size() < 15 && f2.size() < 15 ){
        cout << "Error: Feature Vectors size is < 15" << endl;

    }

    if(relativeSize(f1.at(7), f2.at(7)) < threshold &&
            relativeSize(f1.at(11), f2.at(11)) < threshold &&
            relativeSize(f1.at(15), f2.at(15)) < threshold
            )
    {
        return true;
    }
    else{
        return false;
    }

}

int
main (int argc, char** argv)
{

    try{
        using namespace std;
        using namespace TCLAP;

        //Parsing Cmd-Line
        CmdLine cmd("");

        //directory containing the parts ... in subfolder 1,2,3,4,5, should

        UnlabeledValueArg<std::string> partsPathObject
                ("parts_path_object",
                 "parts path",
                 true, "", "filename", cmd);

        UnlabeledValueArg<std::string> partsPathScene
                ("parts_path_scene",
                 "parts path scene",
                 true, "", "filename", cmd);

        UnlabeledValueArg<std::string> objectFileArg
                ("object_evidence",
                 "Object file. one can try the segmented or non segmented one, as its only used for ICP - see whats better",
                 true, "", "filename", cmd);

        //scene file
        UnlabeledValueArg<std::string> sceneFileArg
                ("scene_evidence",
                 "Scene file.",
                 true, "", "filename", cmd);

        ValueArg<std::string> alignedObjectEvidenceFileArg
                ("", "aligned",
                 "Transformed object evidence, matching object pose.",
                 false, "", "filename", cmd);

        ValueArg<std::string> bestTransfoArg
                ("", "best_transfo",
                 "File to write the most likely transformation to.",
                 false, "", "filename", cmd);

        ValueArg<std::string> outPutSceneArg
                ("", "output_scene",
                 "File to write the most likely transformation to.",
                 false, "", "filename", cmd);


        cmd.parse( argc, argv );



        /////////////////////////////////////////////////
        ////generate datastructures like posnormal///////
        /////////////////////////////////////////////////

        string parts_folder_object = partsPathObject.getValue();
        string parts_folder_scene  = partsPathScene.getValue();

        //        string scene_folder

        vector< vector <float> > feature_vector_object;
        vector< vector <float> > feature_vector_scene;

        getFeatureVector(parts_folder_object, feature_vector_object);
        getFeatureVector(parts_folder_scene, feature_vector_scene);

        /////////////////////////////////////////////////
        /////////////////////////////////////////////////

        //filter based on minimum parts size and eigen-vector similarity and compute prior transformations based on flipping all pca
        //later if 2 eigenvalues are within 5% size or so, add an additional hypothesis

        vector<alignmentHyp> allAlignmentHypothesises;

        cout << "nr of parts of object:" << feature_vector_object.size() << endl;
        cout << "nr of parts of scene :" << feature_vector_scene.size() << endl;


        int nr_correspondences=0;
        for(int i=0; i < feature_vector_object.size(); i++){

            for(int j=0; j < feature_vector_scene.size(); j++){

                vector <float> fo,fs;
                fo=feature_vector_object.at(i);
                fs=feature_vector_scene.at(j);

                //if parts match and their nr of points are larger than a certain threshold (100points in objectpartcloud), create an association hypothesis!
                if(compareFVec(fo,fs,0.5) && fo[16]>50){

                    nr_correspondences++;

                    //create hyp:
                    Eigen::Matrix4f trafo(Eigen::Matrix4f::Identity()), trafoObj(Eigen::Matrix4f::Identity()), trafoScene(Eigen::Matrix4f::Identity());

                    //for all framflips
                    for(int k=0; k < 4; k ++){

                        alignmentHyp trafoHyp;

                        cout << "MATCHED CENTROIDS OBJECT (for debug): " << endl;
                        cout << fo[0]*1000 << " " << fo[1]*1000 << " " << fo[2]*1000 << endl;
                        cout << "MATCHED CENTROIDS SCENE (for debug): " << endl;
                        cout << fs[0]*1000 << " " << fs[1]*1000 << " " << fs[2]*1000 << endl;

                        trafoObj << fo[12] , fo[8], fo[3], fo[0],
                                fo[13] , fo[9], fo[4], fo[1],
                                fo[14] , fo[10],fo[5], fo[2],
                                0      , 0    , 0    , 1    ;

                        trafoScene << fs[12] , fs[8], fs[3], fs[0],
                                fs[13] , fs[9], fs[4], fs[1],
                                fs[14] , fs[10],fs[5], fs[2],
                                0      , 0    , 0    , 1    ;

                        if(k==0) trafoObj=flipFrame(trafoObj, 1, 1, 1);
                        if(k==1) trafoObj=flipFrame(trafoObj,-1, 1,-1);
                        if(k==2) trafoObj=flipFrame(trafoObj, 1,-1,-1);
                        if(k==3) trafoObj=flipFrame(trafoObj,-1,-1, 1);

                        trafo=trafoScene*(trafoObj.inverse());

                        cout << "Trafo for k= " << k << endl;
                        cout << trafo << endl;


                        cout << "in loop nr: " << k << endl;

                        trafoHyp.trafo=trafo;
                        trafoHyp.part_id_obj=fo[17];
                        trafoHyp.part_id_scene=fs[17];

                        allAlignmentHypothesises.push_back(trafoHyp);
                    }
                    cout <<  "size alignment hyp " << allAlignmentHypothesises.size() << endl;

                }
            }
        }

        cout << "TOTAL NR OF CORRESPONDECES: " << nr_correspondences << endl;


        if(nr_correspondences==0){
            cout << "EXIT BECAUSE OF NO CORRESPONDENCES" << endl;
            return EXIT_SUCCESS;
        }

        //doing icp based on full pointclouds and transform prior, debug mode - write all, --best write only one

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

        //Load Pointclouds from PCD File:

        pcl::io::loadPCDFile(objectFileArg.getValue(), *cloud_in);
        pcl::io::loadPCDFile(sceneFileArg.getValue(), *cloud_out);


        cout << "Loading pcd obj: nr points: " << cloud_in->points.size() << endl;
        cout << "Loading pcd scene: nr points: " << cloud_out->points.size() << endl;

        double bestScore=1000;
        Eigen::Matrix4f bestTrafo;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bestPointcloud(new pcl::PointCloud<pcl::PointXYZ>);

        alignmentHyp bestHyp;

        cout << "ALIGNING WITH ICP_REFINEMNT: nr Hypothesis: " << allAlignmentHypothesises.size() << endl;

        for(int i = 0; i < allAlignmentHypothesises.size(); i++){

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::transformPointCloud(*cloud_in, *cloud_transformed, allAlignmentHypothesises[i].trafo );

            cout << "object cloud after transformation: nr points: " << cloud_in->points.size() << endl;

            icp.setInputSource(cloud_transformed); //transfrom cloud in into the scene:
            icp.setInputTarget(cloud_out);

            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
            std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                         icp.getFitnessScore()*1e6 << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;

            stringstream ss;
            ss << alignedObjectEvidenceFileArg.getValue() << i <<".before_icp.pcd";
            //pcl::io::savePCDFileASCII(ss.str(), *cloud_transformed);

            //get minimum score
            if(icp.getFitnessScore()*1e6 < bestScore){

                cout << "better minimum: " << icp.getFitnessScore()*1e6 << endl;
                *bestPointcloud =  Final;
                bestTrafo = icp.getFinalTransformation()*allAlignmentHypothesises[i].trafo;
                bestScore=icp.getFitnessScore()*1e6;
                bestHyp = allAlignmentHypothesises.at(i);


            }

        }

        //set output threshold
        double outPutThreshold=100;
        if(bestScore > outPutThreshold) {
            cout << "EXIT BECAUSE SCORES > " << outPutThreshold << endl;
            return EXIT_SUCCESS;
        }

        if (!bestTransfoArg.getValue().empty())
        {
            //write output to path:
            ofstream of(bestTransfoArg.getValue().c_str());
            if(of.is_open()){
                of << bestScore << endl;
                of << bestTrafo << endl;
                of << bestHyp.part_id_obj << endl;
                of << bestHyp.part_id_scene << endl;
            }
            of.close();
        }
        if (!alignedObjectEvidenceFileArg.getValue().empty())
        {
            pcl::io::savePCDFileASCII(alignedObjectEvidenceFileArg.getValue(), *bestPointcloud);
            //also write scene to current dir


            //pcl::transformPointCloud(*cloud_in, *cloud_in, bestTrafo );
            //pcl::io::savePCDFileASCII(alignedObjectEvidenceFileArg.getValue() + ".before_icp.pcd", *cloud_in);
        }

        if(!outPutSceneArg.getValue().empty())
        {
            pcl::io::savePCDFileASCII(alignedObjectEvidenceFileArg.getValue() + ".scene.pcd", *cloud_out);
        }

        return (0);
    }
    catch (std::exception &e) {
        std::cerr << "Exception caught: ";
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...) {
        std::cerr << "Caught unknown exception." << std::endl;
        return EXIT_FAILURE;
    }


}
