#ifndef BIORBD_TENDONS_TENDON_GEOMETRY_H
#define BIORBD_TENDONS_TENDON_GEOMETRY_H

#include "biorbdConfig.h"

#include <memory>

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
class Vector3d;
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

protected:
  std::shared_ptr<utils::String> f_originSegmentName;
  std::shared_ptr<utils::Vector3d> f_origin;
  std::shared_ptr<utils::String> f_insertionSegmentName;
  std::shared_ptr<utils::Vector3d> f_insertion;
};
}
}
}

#endif  // BIORBD_TENDONS_TENDON_GEOMETRY_H
