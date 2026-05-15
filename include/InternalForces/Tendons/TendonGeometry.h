#ifndef BIORBD_TENDONS_TENDON_GEOMETRY_H
#define BIORBD_TENDONS_TENDON_GEOMETRY_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

#include "InternalForces/Tendons/TendonRoutingPoint.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
class Vector3d;
class Matrix;
}

namespace internal_forces {
namespace tendons {

///
/// \brief Class Geometry of the tendon
///
class BIORBD_API TendonGeometry {
public:
  TendonGeometry();

 ///
 /// @brief Construct tendon geometry
 /// @param originSegmentName The name of the segment where the tendon origin
 /// lies
 /// @param origin The position of the tendon origin on the origin segment
 /// @param insertionSegmentName The name of the segment where the tendon
 /// insertion lies
 /// @param insertion The position of the tendon insertion on the insertion segment
  ///
  TendonGeometry(
    const utils::String& originSegmentName, const utils::Vector3d& origin,
    const utils::String& insertionSegmentName, const utils::Vector3d& insertion);

  TendonGeometry DeepCopy() const;

  void DeepCopy(const TendonGeometry& other);

  void addRoutingPoint(const internal_forces::tendons::TendonRoutingPoint& routingPoint);

  size_t nbRoutingPoints() const;

  TendonRoutingPoint& routingPoint(size_t idx) const;

  const utils::Matrix& lengthsJacobian() const;

protected:
  void computeLengthsJacobian();

  std::shared_ptr<utils::String> m_originSegmentName;
  std::shared_ptr<utils::Vector3d> m_origin;
  std::shared_ptr<utils::String> m_insertionSegmentName;
  std::shared_ptr<utils::Vector3d> m_insertion;
  std::shared_ptr<std::vector<std::shared_ptr<TendonRoutingPoint>>> m_routingPoints; ///< The routing points where the tendon must go through. They are kept and understood in the order of adding.

  std::shared_ptr<utils::Matrix> m_lengthsJacobian;
};
}
}
}

#endif  // BIORBD_TENDONS_TENDON_GEOMETRY_H
