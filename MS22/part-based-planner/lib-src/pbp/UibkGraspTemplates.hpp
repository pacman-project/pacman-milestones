#ifndef UIBKGRASPTEMPLATES_HPP
#define UIBKGRASPTEMPLATES_HPP

#include <nuklei/KernelCollection.h>
#include <nuklei/PoseEstimator.h>
#include <pbp/collission_checker.hpp>
#include <Wm5IntrBox3Box3.h>

namespace pbp {

struct bbox_eigen {
    Eigen::Quaternionf quat;
    Eigen::Vector3f trans;
    Eigen::Vector3f scales;
};
//for colliding bbox
Eigen::Matrix4f mat4FromQuat(const Eigen::Quaternionf & quat,const Eigen::Vector3f trans){

    Eigen::Matrix4f mat4=Eigen::Matrix4f::Identity();
    mat4.block<3,3>(0,0)=quat.toRotationMatrix();
    mat4.col(3) << trans,1;

    return mat4;
}

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

bool checkCollissionBB(const bbox_eigen & bounding_box1, const bbox_eigen & bounding_box2){

    Wm5::Box3<float> box1;
    Wm5::Box3<float> box2;

    Eigen::Matrix4f trafo_bb1=mat4FromQuat(bounding_box1.quat, bounding_box1.trans);
    Eigen::Matrix4f trafo_bb2=mat4FromQuat(bounding_box2.quat, bounding_box2.trans);

    Wm5::Vector3<float> bbox1_x_axis, bbox1_y_axis,bbox1_z_axis;
    Wm5::Vector3<float> bbox2_x_axis, bbox2_y_axis,bbox2_z_axis;
    Wm5::Vector3<float> bbox1_center, bbox2_center;

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

    Wm5::IntrBox3Box3<float> intersectorBoxes(box1,box2);
    return intersectorBoxes.Test();
}


struct UibkGraspTemplates : public nuklei::CustomIntegrandFactor
{

    UibkGraspTemplates(const nuklei::KernelCollection& objectModel,const int & arm_number, robot_state::RobotModelPtr & robot_model); //constructor in implementation

    bool test(const nuklei::kernel::se3& k) const;
    double factor(const nuklei::kernel::se3& k) const;

    bool ikExists(const nuklei::kernel::se3& k, std::vector<double> & jointDsr) const;
    bool trajectoryExists(const  std::vector<double> & jointDsr1, const std::vector<double> & jointDsr) const;
    bool collissionMoveitExists(const  std::vector<double> & jointDsr1) const; //TODO add joints of Hand for selfcollission check

    void setRois(const nuklei::KernelCollection& roiPoses,
                 const std::vector<nuklei::Vector3>& roiWidths);

    void setObstacles(const nuklei::KernelCollection& obstacles);

    void setTablePlane(const nuklei::kernel::se3 plane)
    {
        plane_ = plane;
    }

    void setHandApproachAxis(const int axis)
    {
        if (axis < 0 || axis > 2)
            NUKLEI_THROW("incorrect axis");
        handApproachAxis_ = axis;
    }

    void setBackoffDist(const double d)
    {
        backoffDist_ = d;
    }

    //UIBK stuff
    void setJointsForSDH(const std::vector<double> & hJ)
    {
        handJoints_ = hJ;
    }


    void setBBoxes(const bbox_eigen & bbox_cluster, const bbox_eigen & bbox_template){
        bbox_cluster_ = bbox_cluster;
        bbox_template_ = bbox_template;

    }

    void setArm(const std::string & arm_name){

        if(arm_name=="left_arm")
            arm_nr=1;
        else if (arm_name=="right_arm")
            arm_nr=0;
        else
            NUKLEI_THROW("arm name wrongly defined!");
    }


private:

    nuklei::KernelCollection roiPoses_;
    std::vector<nuklei::Vector3> roiWidths_;
    nuklei::KernelCollection obstacles_;
    std::vector<nuklei::Vector3> roiCorners_;
    boost::optional<nuklei::kernel::se3> plane_;

    int handApproachAxis_;
    double backoffDist_;
    //UIBK stuff
    std::vector<double> handJoints_; //for sdh or Pisa IIT hand
    int arm_nr; //(0 or 1, left and right)
    bbox_eigen bbox_cluster_;
    bbox_eigen bbox_template_;
    //static std::string arm_;

    //For getting contact point of prototype
    nuklei::Vector3 objectCOM_;
    nuklei::KernelCollection objectModel_;
    //robot_state::RobotModelPtr robotModel_; //for self collission checking
    boost::shared_ptr<collisionChecker> collission_obj_Ptr_;

};
}

#endif // UIBKREACHABLEINERTA_HPP
