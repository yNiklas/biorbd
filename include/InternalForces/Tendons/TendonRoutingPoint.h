#ifndef BIORBD_TENDONS_TENDON_ROUTING_POINT_H
#define BIORBD_TENDONS_TENDON_ROUTING_POINT_H

#include "biorbdConfig.h"

#include <memory>

#include "Utils/Vector3d.h"

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace tendons {

///
/// \brief A point where a tendon must go through
///
class BIORBD_API TendonRoutingPoint : public utils::Vector3d {
public:
  TendonRoutingPoint();

  ///
  /// @param x The x coordinate of the routing point's position
  /// @param y The y coordinate of the routing point's position
  /// @param z The z coordinate of the routing point's position
  /// @param name The name of the routing point
  /// @param parentSegmentName The name of the segment where the point is located
  /// @param frictionLoss The friction loss coefficient (between 0 and 1) at the routing point (default is 1.0, meaning no loss)
  ///
  TendonRoutingPoint(
      const utils::Scalar& x,
      const utils::Scalar& y,
      const utils::Scalar& z,
      const utils::String& name,
      const utils::String& parentSegmentName,
      const utils::Scalar& frictionLoss = 1.0);

  TendonRoutingPoint(const utils::Vector3d& other);

  TendonRoutingPoint(const TendonRoutingPoint& other);

  TendonRoutingPoint DeepCopy() const;

  void DeepCopy(const TendonRoutingPoint& other);

  const utils::Scalar& frictionLoss() const;

protected:
  std::shared_ptr<utils::Scalar> m_frictionLoss; ///< The friction loss coefficient at the routing point (between 0 and 1)
};
}
}
}

#endif  // BIORBD_TENDONS_TENDON_ROUTING_POINT_H
