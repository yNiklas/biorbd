#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/Tendon.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::Tendon::Tendon()
    : f_name(std::make_shared<utils::String>()),
      f_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>()) {}

internal_forces::tendons::Tendon::Tendon(
    const utils::String& name,
    const TendonGeometry& geometry)
    : f_name(std::make_shared<utils::String>(name)),
      f_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>(
          geometry)) {}

internal_forces::tendons::Tendon::Tendon(const Tendon& other)
    : f_name(other.f_name),
      f_geometry(other.f_geometry) {}

internal_forces::tendons::Tendon::Tendon(
    const std::shared_ptr<Tendon> other)
    : f_name(other->f_name),
      f_geometry(other->f_geometry) {}

void internal_forces::tendons::Tendon::DeepCopy(
    const Tendon& other) {
  f_name = std::make_shared<utils::String>(*other.f_name);
  f_geometry = std::make_shared<internal_forces::tendons::TendonGeometry>(
      other.f_geometry->DeepCopy());
}
