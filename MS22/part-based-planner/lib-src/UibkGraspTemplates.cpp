#include <pbp/UibkGraspTemplates.hpp>
#include <nuklei/KernelCollection.h>
#include <nuklei/PoseEstimator.h>
#include <pbp/collission_checker.hpp>
#include <pbp/TorqueCIF.hpp>

namespace pbp {

using namespace nuklei;


UibkGraspTemplates::UibkGraspTemplates(const nuklei::KernelCollection& objectModel, const int & arm_number, robot_state::RobotModelPtr & robot_model)
    : objectCOM_(nuklei::Vector3::ZERO),
      objectModel_(objectModel), handApproachAxis_(2), backoffDist_(0), arm_nr(arm_number)
{
    objectCOM_=objectModel_.mean()->getLoc();
    collission_obj_Ptr_.reset(new collisionChecker(robot_model));

    if(arm_nr==0){
        collission_obj_Ptr_->setJointGroup("right_arm");
    }
    else if (arm_nr==1){
        collission_obj_Ptr_->setJointGroup("left_arm");
    }
}

void UibkGraspTemplates::setRois(const KernelCollection &roiPoses, const std::vector<nuklei::Vector3> &roiWidths)
{
    roiPoses_ = roiPoses;
    roiWidths_ = roiWidths;
    roiPoses_.computeKernelStatistics();
    roiCorners_.clear();

    for (unsigned i = 0; i < roiPoses.size(); ++i)
    {
        KernelCollection corners;
        Vector3 e = roiWidths.at(i);

        {
            kernel::r3 k;
            k.loc_ = Vector3(e.X()/2 , + e.Y()/2 , + e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(e.X()/2 , + e.Y()/2 , - e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(e.X()/2 , - e.Y()/2 , + e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(e.X()/2 , - e.Y()/2 , - e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(- e.X()/2 , + e.Y()/2 , + e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(- e.X()/2 , + e.Y()/2 , - e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(- e.X()/2 , - e.Y()/2 , + e.Z()/2);
            corners.add(k);
        }
        {
            kernel::r3 k;
            k.loc_ = Vector3(- e.X()/2 , - e.Y()/2 , - e.Z()/2);
            corners.add(k);
        }

        kernel::se3 pose(roiPoses.at(i));
        corners.transformWith(pose.loc_, pose.ori_);
        for (KernelCollection::iterator kci = corners.begin();
             kci != corners.end(); ++kci)
            roiCorners_.push_back(kci->getLoc());
    }

}

void UibkGraspTemplates::setObstacles(const nuklei::KernelCollection& obstacles)
{
  using namespace nuklei;
  if (obstacles.size() > 0)
  {
    for (KernelCollection::const_sample_iterator i = obstacles.sampleBegin(50);
         i != i.end(); ++i)
      obstacles_.add(*i);
    obstacles_.uniformizeWeights();
    obstacles_.computeKernelStatistics();
  }
}

bool UibkGraspTemplates::test(const nuklei::kernel::se3& k) const
{

  //check if inside boundingbox!
    Eigen::Quaternionf quat(k.ori_.W(),k.ori_.X(),k.ori_.Y(),k.ori_.Z() );
    Eigen::Vector3f trans(k.loc_.X(), k.loc_.Y(), k.loc_.Z());
    Eigen::Matrix4f trafo=mat4FromQuat(quat,trans);

    if(!checkCollissionBB(transformBB(bbox_template_,trafo), bbox_cluster_))
        return false;
        ///////

  // Check against upward grasp.
  // This assumes that Z is pointing up.
  Vector3 kori(la::normalized(la::matrixCopy(k.ori_).GetColumn(2)));
//  Vector3 kori(la::normalized(la::matrixCopy(k.ori_).GetColumn(handApproachAxis_)));

  if (kori.Dot(Vector3(0,0,-1)) < -.2)
    return false;

  // Check that there's no collision with plane

  if (plane_ && roiCorners_.size() > 0)
  {
    kernel::se3 tplane = plane_->transformedWith(k.inverseTransformation());
    Vector3 normal = la::normalized(la::matrixCopy(tplane.ori_).GetColumn(2));
    if (la::matrixCopy(plane_->ori_).GetColumn(2)[2] < 0)
      normal = -normal;
    Plane3 p(normal,
             tplane.loc_);
    for (std::vector<Vector3>::const_iterator i = roiCorners_.begin();
         i != roiCorners_.end(); ++i)
    {
      if (p.WhichSide(*i) < 0)
        return false;
    }
  }



  // Check for IK

  std::vector<double> jntPosDsr, jntPosDsr_approach, jntPosDsr_retreat;
  jntPosDsr.resize(7);
  jntPosDsr_approach.resize(7);
  jntPosDsr_retreat.resize(7);

  if (!ikExists(k, jntPosDsr))
    return false;

  //if (backoffDist_ > 0)
  //{
    // Check for IK for backup pose
    kernel::se3 kk = k;
    kk.loc_ += -kori*100; //100 mm should be good!
    if (!ikExists(kk,jntPosDsr_approach))
      return false;


  //Check for IK for retreat pose //10 cm upwards
  kernel::se3 kkk = k;
  kkk.loc_.Z() += 100; //mm
  if (!ikExists(kkk,jntPosDsr_retreat))
    return false;

  //}

  //Check that the solutions are smooth!
  if(!trajectoryExists(jntPosDsr, jntPosDsr_approach))
      return false;

  if(!trajectoryExists(jntPosDsr, jntPosDsr_retreat))
      return false;

  //Check that there is no self-collission - moveit:

  //if(collissionMoveitExists(jntPosDsr))
  //    return false;

  //if(collissionMoveitExists(jntPosDsr_retreat))
  //    return false;


  // Check that there's no collision
  if (roiPoses_.size() > 0)
  {
    KernelCollection tmpRoiPoses = roiPoses_;
    tmpRoiPoses.transformWith(k);
    boost::shared_ptr<RegionOfInterest> r;
    for (unsigned i = 0; i < tmpRoiPoses.size(); ++i)
    {
      kernel::se3 kk = kernel::se3(tmpRoiPoses.at(i));
      boost::shared_ptr<RegionOfInterest> p(new BoxROI(kk.loc_, kk.ori_, roiWidths_.at(i)));
      if (r) r->enqueue(p);
      else r = p;
    }
    KernelCollection::const_iterator i = obstacles_.begin();
    for (; i != obstacles_.end(); ++i)
      if (r->contains(i->getLoc())) return false;
  }

//  if(collissionMoveitExists(jntPosDsr_approach))
//      return false; //most important as this is the first pose to plan for :)

  return true;
}

double UibkGraspTemplates::factor(const nuklei::kernel::se3& pose) const
{
  // returns a number between 1 and 1.2.
  // returns 1 if torque is large
  // returns 1.2 if torque is low

  Matrix3 m = la::matrixCopy(pose.ori_);
  Vector3 handZAxis = m.GetColumn(2);

  Vector3 contactPoint = pose.loc_ + handZAxis*std::fabs(objectCOM_.Z()); // contact point 180mm forward from TCP
  //for now use com

//  Vector3 sceneCOM;
//  sceneCOM.X()=com_[0];
//  sceneCOM.Y()=com_[1];
//  sceneCOM.Z()=com_[2];

//  Vector3 r = sceneCOM - contactPoint;
//  Vector3 F = -Vector3::UNIT_Z;

//  double torque = r.Cross(F).Length();
//  double fact = std::max(0., 4*(100-torque) / 1000);

#if 0
  if (!outfile.empty())
  {
    std::ofstream ofs(outfile.c_str());
    ofs << NUKLEI_NVP(handZAxis) << std::endl;
    ofs << NUKLEI_NVP(contactPoint) << std::endl;
    ofs << NUKLEI_NVP(r) << std::endl;
    ofs << NUKLEI_NVP(F) << std::endl;
    ofs << NUKLEI_NVP(torque) << std::endl;
    ofs << NUKLEI_NVP(fact) << std::endl;
    ofs << NUKLEI_NVP(1+fact) << std::endl;
  }
#endif

  return 1;//+fact;
}


}

//Implementation of IK stuffs and collissions
#include "UIBK_kinematics.hpp"

namespace pbp {

  using namespace nuklei;
  using namespace Eigen;

  bool UibkGraspTemplates::ikExists(const nuklei::kernel::se3& k, std::vector<double> & jointDsr) const
  {

      //TODO right is not correct anymore, you can set the arm by the interface

    Eigen::Affine3d kuka_right_pose;
    Translation<double,3> kuka_right_pose_trans;
    Quaterniond kuka_right_pose_rotation;


    //k is the pose in world frame, the solver works in the worldframe!
    //Transform to eigen

    kuka_right_pose_trans.x()=k.loc_.X()*0.001; //transform to meters
    kuka_right_pose_trans.y()=k.loc_.Y()*0.001;
    kuka_right_pose_trans.z()=k.loc_.Z()*0.001;

    kuka_right_pose_rotation.w()=k.ori_.W();
    kuka_right_pose_rotation.x()=k.ori_.X();
    kuka_right_pose_rotation.y()=k.ori_.Y();
    kuka_right_pose_rotation.z()=k.ori_.Z();

   UIBK_kinematics::UIBK_lwr_kinematics uibk_kinematics; //should be statically called, we'll see the performance :D

    //setting arm:
   if(arm_nr==0)
       uibk_kinematics.setArm("right_arm");
   else if (arm_nr==1)
       uibk_kinematics.setArm("left_arm");

    kuka_right_pose =kuka_right_pose_trans*kuka_right_pose_rotation;
    bool ik_exists;
    ik_exists=uibk_kinematics.ikSolver(jointDsr, kuka_right_pose, jointDsr);

    return ik_exists;
  }

  bool UibkGraspTemplates::trajectoryExists(const std::vector<double> & jointDsr1,const std::vector<double> & jointDsr2 ) const
  {
    bool trajectoryExists;
    double jointThreshold=M_PI; /////PARAMETER FOR JOINT CHANGES, EXPERIENCE SHOULDNT CHANGE MORE THAN 1 RADIAN
    double metric=0;
    for (int i =0; i < 7; i++){
        metric += fabs(jointDsr1.at(i) - jointDsr2.at(i));
    }

    trajectoryExists = (metric  < jointThreshold);

    return trajectoryExists;
  }

  //Selfcollission in moveit, also collision with the table is included a second time TODO: add joints for hand!

  bool UibkGraspTemplates::collissionMoveitExists(const  std::vector<double> & jointDsr1) const{



      //setting the name of the joint group
      return collission_obj_Ptr_->is_colliding_scene(jointDsr1);

  }
}//pbp
