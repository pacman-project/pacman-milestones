#ifndef PBP_REACHABLE_HPP
#define PBP_REACHABLE_HPP

#include <nuklei/KernelCollection.h>
#include <nuklei/PoseEstimator.h>

namespace pbp {
  
  struct Reachable : public nuklei::CustomIntegrandFactor
  {
    Reachable();
    
    bool test(const nuklei::kernel::se3& k) const;
    double factor(const nuklei::kernel::se3& k) const { return 1.; }
    
    virtual bool
    ikExists(const nuklei::kernel::se3& k) const { return true; }
    
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
    
  private:
    nuklei::KernelCollection roiPoses_;
    std::vector<nuklei::Vector3> roiWidths_;
    nuklei::KernelCollection obstacles_;
    std::vector<nuklei::Vector3> roiCorners_;
    boost::optional<nuklei::kernel::se3> plane_;
    int handApproachAxis_;
    double backoffDist_;
  };
  
}
#endif
