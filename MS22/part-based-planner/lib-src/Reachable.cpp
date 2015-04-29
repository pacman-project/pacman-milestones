#include <pbp/Reachable.hpp>

namespace pbp {
  
  using namespace nuklei;
  
  Reachable::Reachable() :
  handApproachAxis_(2), backoffDist_(0)
  {}
  
  void Reachable::setRois(const nuklei::KernelCollection& roiPoses,
                          const std::vector<nuklei::Vector3>& roiWidths)
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
  
  void Reachable::setObstacles(const nuklei::KernelCollection& obstacles)
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

  bool Reachable::test(const nuklei::kernel::se3& k) const
  {
    // Check against upward grasp.
    // This assumes that Z is pointing up.
    Vector3 kori(la::normalized(la::matrixCopy(k.ori_).GetColumn(handApproachAxis_)));
    if (kori.Dot(Vector3(0,0,-1)) < -.2)
      return false;

    // Check that there's no collision
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
    if (!ikExists(k))
      return false;
    
    if (backoffDist_ > 0)
    {
      // Check for IK for backup pose
      kernel::se3 kk = k;
      kk.loc_ += -kori*backoffDist_;
      if (!ikExists(kk))
        return false;
    }

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

    return true;
  }
  
}
