//Version 1.0 - 10.11.14 - added global ik function which calls arm dependent on setting
#ifndef UIBK_KINEMATICS_HPP_
#define UIBK_KINEMATICS_HPP_

#include "KukaLWR_Kinematics.hpp"
#include <Eigen/Dense>
// #include <Eigen/Geometry>
//Class wrapper for ETH Zurich LWR kinematics suited for our robots

using namespace std;
using namespace KDL;
using namespace Eigen;

#define DEBUG_OUTPUT 0

namespace UIBK_kinematics{

  class UIBK_lwr_kinematics {
    
  private:
    
   kuka_IK::KukaLWR_Kinematics * KukaLWRPtr_;
   
   Eigen::Affine3d eef_transform_right_;
   Eigen::Affine3d eef_transform_left_;
   
   Eigen::Affine3d eef_transform_right_inverse_;
   Eigen::Affine3d eef_transform_left_inverse_;
   
   Eigen::Affine3d eef_world_link_to_right_arm_base_link_;
   Eigen::Affine3d eef_world_link_to_left_arm_base_link_;
   
   Eigen::Affine3d eef_world_link_to_right_arm_base_link_inverse_;
   Eigen::Affine3d eef_world_link_to_left_arm_base_link_inverse_;
   
   Eigen::Affine3d tmp_pose_right_;
   Eigen::Affine3d tmp_pose_left_;

   std::string current_arm_;
   
   
   int ikSolverNr_; //1 - ik_cf_6dof; 2- ik_cf_7dof; 3 -ik_iterative;
   void init_() {
     
     ikSolverNr_ = 1;
     current_arm_ = "right_arm";
    
     Eigen::Affine3d world_base_right;
     Eigen::Affine3d base_link_2_link;
     Eigen::Affine3d world_base_left;
     Eigen::Affine3d helper_rotation;
     
     
     world_base_right = Translation<double,3>(-0.496, 0.599,0.633)*Quaterniond(0.475, 0.798, -0.186,0.322); //got from URDF file
     world_base_left = Translation<double,3>(-0.492, 0.754,0.629)*Quaterniond(0.564, -0.788, 0.242,-0.040);
     base_link_2_link = Translation<double,3>(0, 0,0.310)*Quaterniond(1,0,0,0);
     helper_rotation = AngleAxisd(M_PI, Vector3d::UnitZ());
     
     #if DEBUG_OUTPUT
	cout << "world\n" << world_base_right.matrix() << endl;
	cout << "helperrot\n" << helper_rotation.matrix() << endl;
	cout << "baseToLink\n" << base_link_2_link.matrix() << endl;
      #endif
	
     eef_world_link_to_right_arm_base_link_ = world_base_right * helper_rotation * base_link_2_link;
     #if DEBUG_OUTPUT
	cout << "DebugMat\n" << eef_world_link_to_right_arm_base_link_.matrix() << endl;
	cout << "DebugMat\n" << eef_world_link_to_right_arm_base_link_.matrix() << endl;
      #endif
     eef_world_link_to_left_arm_base_link_ = world_base_left * helper_rotation * base_link_2_link;
     
     eef_transform_right_ = Eigen::Affine3d::Identity();
     eef_transform_left_ = Eigen::Affine3d::Identity();
     eef_transform_right_inverse_ = Eigen::Affine3d::Identity();
     eef_transform_left_inverse_ = Eigen::Affine3d::Identity();
     
     
     eef_world_link_to_right_arm_base_link_inverse_ = eef_world_link_to_right_arm_base_link_.inverse();
     eef_world_link_to_left_arm_base_link_inverse_ = eef_world_link_to_left_arm_base_link_.inverse();
   }
      
  public:
    
    //sets a global transform on the endeffector like tool offset    
    void setRightArmEndeffectorTransform(const Eigen::Affine3d & eef_offset){
      eef_transform_right_ = eef_offset;
      eef_transform_right_inverse_=eef_transform_right_.inverse();
    }
    void setLeftArmEndeffectorTransform(const Eigen::Affine3d & eef_offset){
      eef_transform_left_ = eef_offset;
      eef_transform_left_inverse_=eef_transform_left_.inverse();
    }
    void setRightArmWorldTransform(const Eigen::Affine3d & world_transform){
      eef_world_link_to_right_arm_base_link_ = world_transform;
      eef_world_link_to_right_arm_base_link_inverse_=eef_world_link_to_right_arm_base_link_.inverse();
    }
    void setLeftArmWorldTransform(const Eigen::Affine3d & world_transform){
      eef_world_link_to_left_arm_base_link_ = world_transform;
      eef_world_link_to_left_arm_base_link_inverse_ = eef_world_link_to_left_arm_base_link_.inverse();
    }
    
    void setSolver(const string & solverName){
      if (solverName == "ik_cf_6dof")
	ikSolverNr_=1;
      else if (solverName == "ik_cf_7dof")
	ikSolverNr_=2;
      else if (solverName == "ik_iterative")
	ikSolverNr_=3;
      else if (solverName == "debug")
	ikSolverNr_=-1;
      else 
	cout << "Failed to change solver settings: allowed: ";
	cout  << " 1 - ik_cf_6dof " << " 2 - ik_cf_7dof (to be tested) " << " 3 - ik_iterative (to be tested) " <<  endl;
	     
      cout << "IkSolver (still) set to Nr: " << ikSolverNr_ << endl;
    }
    
    void printStatus(){
      cout << "Solver set to Nr: \n" << ikSolverNr_ << endl;
      
      cout << "TF_World_to_Right_Base: \n" << eef_world_link_to_right_arm_base_link_.matrix() << endl;
      cout << "TF_World_to_Left_Base: \n" << eef_world_link_to_left_arm_base_link_.matrix() << endl;
      cout << "TF_eef_right_tf: \n" << eef_transform_right_.matrix() << endl;
      cout << "TF_eef_left_tf: \n" << eef_transform_right_.matrix() << endl;
    }
    
    //Inverse Kinematics
    bool ikSolverRightArm(const vector<double> & jntPosMsr,const Eigen::Affine3d & poseDsr, vector<double> & jntPosDsr){
      //Pose To UIBK Pose:
      #if DEBUG_OUTPUT
	cout << "Pose before (109)\n" << poseDsr.matrix() << endl;
	cout << "WF-1\n" << eef_world_link_to_right_arm_base_link_.inverse().matrix() << endl;
	cout << "EF-1\n" << eef_transform_right_.inverse().matrix() << endl;
	cout << "WF-1*WF\n" << eef_world_link_to_right_arm_base_link_.inverse().matrix() * eef_world_link_to_right_arm_base_link_.matrix() << endl;
// 	cout << "DebugMat\n" << eef_world_link_to_right_arm_base_link_.matrix() << endl;
      #endif
      tmp_pose_right_ = eef_world_link_to_right_arm_base_link_inverse_ * poseDsr * eef_transform_right_inverse_;
      #if DEBUG_OUTPUT
	cout << "TmpPose (109)\n" << tmp_pose.matrix() << endl;
// 	cout << "DebugMat\n" << eef_world_link_to_right_arm_base_link_.matrix() << endl;
      #endif
      if (ikSolverNr_ == 1) {
	return (*KukaLWRPtr_).ikSolver(jntPosDsr, tmp_pose_right_ ,jntPosDsr);
      }
      else if (ikSolverNr_ == 2) {
	return (*KukaLWRPtr_).ikSolverAnalytical7DOF(tmp_pose_right_,jntPosDsr);
      }
      else if (ikSolverNr_ == 3) {
	vector<double> jntPosMsr2 = jntPosMsr; //copy to remove const
	return (*KukaLWRPtr_).ikSolverIterative7DOF(jntPosMsr2, tmp_pose_right_,jntPosDsr);
      }
      else if (ikSolverNr_ == -1) {
        
	return true; //grasp planner debug mode
      }
      else 
	cout << "[Error:] Ik Solver wrongly set!" << endl;
      return false;           
    }
    
    bool ikSolverLeftArm(const vector<double> & jntPosMsr,const Eigen::Affine3d & poseDsr, vector<double> & jntPosDsr){
      //Pose To UIBK Pose:
      tmp_pose_left_ = eef_world_link_to_left_arm_base_link_inverse_ * poseDsr * eef_transform_left_inverse_;
      
      if (ikSolverNr_ == 1) {
	return (*KukaLWRPtr_).ikSolver(jntPosDsr, tmp_pose_left_,jntPosDsr);
      }
      else if (ikSolverNr_ == 2) {
	return (*KukaLWRPtr_).ikSolverAnalytical7DOF(tmp_pose_left_,jntPosDsr);
      }
      else if (ikSolverNr_ == 3) {      
	return (*KukaLWRPtr_).ikSolverAnalytical7DOF(tmp_pose_left_,jntPosDsr);
      }
      else if (ikSolverNr_ == -1) {
      
	return true; //grasp planner debug mode - all poses are reachable
      }
      else 
	cout << "[Error:] Ik Solver wrongly set!" << endl;
      return false;           
    }
    //Forward Kinematics
    bool fkSolverRightArm(const vector<double> & jntPosDsr, Eigen::Affine3d & poseDsr){
      
      
	
      bool fkRet = (*KukaLWRPtr_).fkSolver(jntPosDsr, poseDsr);
      #if DEBUG_OUTPUT
	cout << "Pose before: \n" << poseDsr.matrix() << endl; 
      #endif
	
      poseDsr = eef_world_link_to_right_arm_base_link_.matrix()*poseDsr.matrix()*eef_transform_right_.matrix();
      
	 #if DEBUG_OUTPUT
	 
	cout << endl << "Trafo: \n" << eef_world_link_to_right_arm_base_link_.matrix() << endl; 
	cout << "Pose after: \n" << poseDsr.matrix() << endl; 
      #endif
      return fkRet;
    }
    
    bool fkSolverLeftArm(const vector<double> & jntPosDsr, Eigen::Affine3d & poseDsr){
      bool fkRet = (*KukaLWRPtr_).fkSolver(jntPosDsr, poseDsr);
       #if DEBUG_OUTPUT
	cout << "Pose before: \n" << poseDsr.matrix() << endl; 
      #endif
      poseDsr = eef_world_link_to_left_arm_base_link_.matrix()*poseDsr.matrix()*eef_transform_left_.matrix();
      #if DEBUG_OUTPUT
	 
	cout << endl << "Trafo: \n" << eef_world_link_to_left_arm_base_link_.matrix() << endl; 
	cout << "Pose after: \n" << poseDsr.matrix() << endl; 
      #endif
      return fkRet;
    }

    void setArm(const std::string & arm){
        if(arm=="right_arm")
            current_arm_=arm;
        else if(arm=="left_arm")
            current_arm_=arm;
        else std::cout << "Error: in Arm name, using default-right" << std::endl;

    }

    bool ikSolver(const vector<double> & jntPosMsr,const Eigen::Affine3d & poseDsr, vector<double> & jntPosDsr){
        if(current_arm_=="right_arm")
            return ikSolverRightArm(jntPosMsr,poseDsr, jntPosDsr);
        else if(current_arm_=="left_arm")
            return ikSolverLeftArm(jntPosMsr,poseDsr, jntPosDsr);
        else
            return false;
    }

    bool fkSolver(const vector<double> & jntPosDsr, Eigen::Affine3d & poseDsr){
        if(current_arm_=="right_arm")
            return fkSolverRightArm(jntPosDsr, poseDsr);
        else if (current_arm_=="left_arm")
            return fkSolverRightArm(jntPosDsr, poseDsr);
        else
            return false;
    }
    
    
    UIBK_lwr_kinematics() {
      KukaLWRPtr_ = new kuka_IK::KukaLWR_Kinematics;
      init_();
      
    }  //Initializer List
    
  };
  

}//NS

#endif /* UIBK_KINEMATICS_HPP_ */
