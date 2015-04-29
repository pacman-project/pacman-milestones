#ifndef PBP_UIBKRIGHTKUKAREACHABLE_H
#define PBP_UIBKRIGHTKUKAREACHABLE_H

#include <nuklei/KernelCollection.h>
#include <nuklei/PoseEstimator.h>
//#include <pbp/collission_checker.hpp>

namespace pbp {

struct UibkRightKukaReachable : public nuklei::CustomIntegrandFactor
{
  UibkRightKukaReachable(const nuklei::KernelCollection& roiPoses,
                const std::vector<nuklei::Vector3>& roiWidths,
                const nuklei::KernelCollection& obstacles);//, const robot_state::RobotModelPtr & model);

  bool test(const nuklei::kernel::se3& k) const;
  double factor(const nuklei::kernel::se3& k) const { return 1.; }

  static bool ikExists(const nuklei::kernel::se3& k, std::vector<double> & jointDsr);
  static bool trajectoryExists(const  std::vector<double> & jointDsr1, const std::vector<double> & jointDsr);
  //bool collissionMoveitExists(const  std::vector<double> & jointDsr1);

private:
  nuklei::KernelCollection roiPoses_;
  std::vector<nuklei::Vector3> roiWidths_;
  nuklei::KernelCollection obstacles_;
  //robot_state::RobotModelPtr model_;
};

} //pbp



#endif // PBP_UIBKRIGHTKUKAREACHABLE_H
