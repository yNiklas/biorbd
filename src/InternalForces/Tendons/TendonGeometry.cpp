#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/TendonGeometry.h"

#include "Utils/String.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::TendonGeometry::TendonGeometry()
    : f_originSegmentName(std::make_shared<utils::String>()),
      f_origin(std::make_shared<utils::Vector3d>()),
      f_insertionSegmentName(std::make_shared<utils::String>()),
      f_insertion(std::make_shared<utils::Vector3d>()) {}

internal_forces::tendons::TendonGeometry::TendonGeometry(
    const utils::String& originSegmentName,
    const utils::Vector3d& origin,
    const utils::String& insertionSegmentName,
    const utils::Vector3d& insertion)
    : f_originSegmentName(std::make_shared<utils::String>(originSegmentName)),
      f_origin(std::make_shared<utils::Vector3d>(origin)),
      f_insertionSegmentName(
          std::make_shared<utils::String>(insertionSegmentName)),
      f_insertion(std::make_shared<utils::Vector3d>(insertion)) {}

internal_forces::tendons::TendonGeometry
internal_forces::tendons::TendonGeometry::DeepCopy() const {
  internal_forces::tendons::TendonGeometry copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::tendons::TendonGeometry::DeepCopy(
    const TendonGeometry& other) {
  f_originSegmentName = std::make_shared<utils::String>(*other.f_originSegmentName);
  f_origin = std::make_shared<utils::Vector3d>(*other.f_origin);
  f_insertionSegmentName = std::make_shared<utils::String>(*other.f_insertionSegmentName);
  f_insertion = std::make_shared<utils::Vector3d>(*other.f_insertion);
}
