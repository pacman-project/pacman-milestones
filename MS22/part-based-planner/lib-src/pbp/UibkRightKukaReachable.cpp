#include <pbp/UibkRightKukaReachable.hpp>
//#include <pbp/collission_checker.hpp>

namespace pbp {

  using namespace nuklei;

  UibkRightKukaReachable::UibkRightKukaReachable(const nuklei::KernelCollection& roiPoses,
                               const std::vector<nuklei::Vector3>& roiWidths,
                               const nuklei::KernelCollection& obstacles) //, const robot_state::RobotModelPtr & model) :
      :
      roiPoses_(roiPoses), roiWidths_(roiWidths)//, model_(model)
  {
    using namespace nuklei;
    roiPoses_.computeKernelStatistics();
    for (KernelCollection::const_sample_iterator i = obstacles.sampleBegin(300);
         i != i.end(); ++i)
      obstacles_.add(*i);
    obstacles_.uniformizeWeights();
    obstacles_.computeKernelStatistics();

  }

  bool UibkRightKukaReachable::test(const nuklei::kernel::se3& k) const
  {

    // Check against upward grasp.
    // This assumes that X is pointing away from the palm.
      std::vector<double> jntPosDsr, jntPosDsr_approach, jntPosDsr_retreat;
      jntPosDsr.resize(7);
      jntPosDsr_approach.resize(7);
      jntPosDsr_retreat.resize(7);

    Vector3 kori(la::normalized(la::matrixCopy(k.ori_).GetColumn(0)));
    if (kori.Dot(Vector3(0,0,-1)) < -.2)
      return false;

    // Check for IK
    if (!ikExists(k,jntPosDsr))
      return false;


    // Check for IK for backup pose //10 cm backwards
    kernel::se3 kk = k;
    kk.loc_ += -kori*100; //mm
    if (!ikExists(kk, jntPosDsr_approach))
      return false;

    //Check for IK for retreat pose //10 cm upwards
    kernel::se3 kkk = k;
    kkk.loc_.Z() += 100; //mm
    if (!ikExists(kkk,jntPosDsr_retreat))
      return false;

    //Check that the solutions are smooth!
    if(!trajectoryExists(jntPosDsr, jntPosDsr_approach))
        return false;

    if(!trajectoryExists(jntPosDsr, jntPosDsr_retreat))
        return false;



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
      //#define PBP_REACHABLE_STOC
#ifdef PBP_REACHABLE_STOC
      KernelCollection::const_sample_iterator i =
      as_const(obstacles_).sampleBegin(std::min(as_const(obstacles_).size(),
                                                size_t(2000)));
      for (; i != i.end(); ++i)
        if (r->contains(i->getLoc())) return false;
#else
      KernelCollection::const_iterator i = obstacles_.begin();
      for (; i != obstacles_.end(); ++i)
        if (r->contains(i->getLoc())) return false;
#endif
    }

#if 0
    //dump pose and kinematics info
    std::ofstream file;
    file.open("/tmp/pbp-correct_poses.txt", std::ios::out|std::ios::app);
    //
    if (file.is_open()){
        file << k.loc_.X() << " " << k.loc_.Y() << " " << k.loc_.Z() << " ";
        file << k.ori_.W() << " " << k.ori_.X() << " " << k.ori_.Y() << " " << k.ori_.Z() << " jt: ";
        file << jntPosDsr.at(0) << " " << jntPosDsr.at(1) << " " << jntPosDsr.at(2) << " " << jntPosDsr.at(3) << " " << jntPosDsr.at(4) << " " << jntPosDsr.at(5) << " " << jntPosDsr.at(6) << std::endl;

        file << kk.loc_.X() << " " << kk.loc_.Y() << " " << kk.loc_.Z() << " ";
        file << kk.ori_.W() << " " << kk.ori_.X() << " " << kk.ori_.Y() << " " << kk.ori_.Z() << " jt: ";
        file << jntPosDsr_approach.at(0) << " " << jntPosDsr_approach.at(1) << " " << jntPosDsr_approach.at(2) << " " << jntPosDsr_approach.at(3) << " " << jntPosDsr_approach.at(4) << " " << jntPosDsr_approach.at(5) << " " << jntPosDsr_approach.at(6) << std::endl;

        file << kkk.loc_.X() << " " << kkk.loc_.Y() << " " << kkk.loc_.Z() << " ";
        file << kkk.ori_.W() << " " << kkk.ori_.X() << " " << kkk.ori_.Y() << " " << kkk.ori_.Z() << " jt: ";
        file << jntPosDsr_retreat.at(0) << " " << jntPosDsr_retreat.at(1) << " " << jntPosDsr_retreat.at(2) << " " << jntPosDsr_retreat.at(3) << " " << jntPosDsr_retreat.at(4) << " " << jntPosDsr_retreat.at(5) << " " << jntPosDsr_retreat.at(6) << std::endl;

        file << std::endl;
    }
    file.close();
#endif
    return true;
  }
}





#include "UIBK_kinematics.hpp"

namespace pbp {

  using namespace nuklei;
  using namespace Eigen;

  bool UibkRightKukaReachable::ikExists(const nuklei::kernel::se3& k, std::vector<double> & jointDsr)
  {
//    double T[16];

    Eigen::Affine3d kuka_right_pose;
    Translation<double,3> kuka_right_pose_trans;
    Quaterniond kuka_right_pose_rotation;


    kernel::se3 pbp_pose = k; //This is the pose in world frame, the solver works in the worldframe!
//    kernel::se3 pbp_pose = k;
//    Vector3 kori(la::normalized(la::matrixCopy(k.ori_).GetColumn(0)));
//    pbp_pose.loc_ += kori*.05; //in meters??

    //set offset and rotate pose for 90 degree - to be testet!

    //Transform to eigen

    kuka_right_pose_trans.x()=pbp_pose.loc_.X()*0.001; //transform to meters
    kuka_right_pose_trans.y()=pbp_pose.loc_.Y()*0.001;
    kuka_right_pose_trans.z()=pbp_pose.loc_.Z()*0.001;

    kuka_right_pose_rotation.w()=pbp_pose.ori_.W();
    kuka_right_pose_rotation.x()=pbp_pose.ori_.X();
    kuka_right_pose_rotation.y()=pbp_pose.ori_.Y();
    kuka_right_pose_rotation.z()=pbp_pose.ori_.Z();

//    right_eef_offset = AngleAxisd(M_PI/2, Vector3d::UnitZ()); //not needed as prototypes are stored in the correct format


    UIBK_kinematics::UIBK_lwr_kinematics uibk_kinematics; //should be statically called, we'll see the performance :D

//    uibk_kinematics.setRightArmEndeffectorTransform(right_eef_offset);
    kuka_right_pose =kuka_right_pose_trans*kuka_right_pose_rotation;
    bool ik_exists;
    ik_exists=uibk_kinematics.ikSolverRightArm(jointDsr, kuka_right_pose, jointDsr);

    return ik_exists;
  }

  bool UibkRightKukaReachable::trajectoryExists(const std::vector<double> & jointDsr1,const std::vector<double> & jointDsr2 )
  {
    bool trajectoryExists;
    double jointThreshold=M_PI;
    double metric=0;
    for (int i =0; i < 7; i++){
        metric += fabs(jointDsr1.at(i) - jointDsr2.at(i));
    }

    trajectoryExists = (metric  < jointThreshold);

    return trajectoryExists;
  }

//  bool UibkRightKukaReachable::collissionMoveitExists(const  std::vector<double> & jointDsr1){

//      collisionChecker collission_obj(model_);
//      return collission_obj.is_colliding_scene(jointDsr1);

//  }

}




