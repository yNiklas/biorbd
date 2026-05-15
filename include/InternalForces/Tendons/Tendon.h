#ifndef BIORBD_TENDON_H
#define BIORBD_TENDON_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

#include "InternalForces/Tendons/TendonGeometry.h"
#include "InternalForces/Tendons/TendonRoutingPoint.h"

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace tendons {
class Tendons;

class BIORBD_API Tendon {
  friend Tendons;

public:
  Tendon();

 ///
 /// \brief Construct a tendon
 /// @param name Name of the tendon
 /// @param geometry Position and segments of origin and insertion
  ///
  Tendon(
    const utils::String& name,
    const TendonGeometry& geometry);

  Tendon(const Tendon& other);

  Tendon(const std::shared_ptr<Tendon> other);

  void DeepCopy(const Tendon& other);

  void addRoutingPoint(const internal_forces::tendons::TendonRoutingPoint& routingPoint);

  size_t nbRoutingPoints() const;

  TendonRoutingPoint& routingPoint(size_t idx) const;

protected:
  std::shared_ptr<utils::String> m_name;
  std::shared_ptr<internal_forces::tendons::TendonGeometry> m_geometry;
  std::shared_ptr<std::vector<std::shared_ptr<TendonRoutingPoint>>> m_routingPoints; ///< The routing points where the tendon must go through. They are kept and understood in the order of adding.
};
}
}
}

#endif  // BIORBD_TENDON_H
