#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/Tendon.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::Tendon::Tendon()
    : m_name(std::make_shared<utils::String>()),
      m_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>()) {}

internal_forces::tendons::Tendon::Tendon(
    const utils::String& name,
    const TendonGeometry& geometry)
    : m_name(std::make_shared<utils::String>(name)),
      m_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>(geometry)) {}

internal_forces::tendons::Tendon::Tendon(const Tendon& other)
    : m_name(other.m_name),
      m_geometry(other.m_geometry) {}

internal_forces::tendons::Tendon::Tendon(
    const std::shared_ptr<Tendon> other)
    : m_name(other->m_name),
      m_geometry(other->m_geometry) {}

void internal_forces::tendons::Tendon::DeepCopy(
    const Tendon& other) {
  m_name = std::make_shared<utils::String>(*other.m_name);
  m_geometry = std::make_shared<internal_forces::tendons::TendonGeometry>(
      other.m_geometry->DeepCopy());
}

void internal_forces::tendons::Tendon::addRoutingPoint(
  const TendonRoutingPoint& routingPoint,
  const utils::Scalar& sectionFrictionLoss) {
  m_geometry->addRoutingPoint(routingPoint, sectionFrictionLoss);
}

size_t internal_forces::tendons::Tendon::nbRoutingPoints() const {
  return m_geometry->nbRoutingPoints();
}

size_t internal_forces::tendons::Tendon::nbSections() const {
  return m_geometry->nbRoutingPoints() + 1;
}

internal_forces::tendons::TendonRoutingPoint&
internal_forces::tendons::Tendon::routingPoint(size_t idx) const {
  return m_geometry->routingPoint(idx);
}

const internal_forces::tendons::TendonGeometry& internal_forces::tendons::Tendon::geometry() const {
  return *m_geometry;
}

void internal_forces::tendons::Tendon::updateKinematics(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) {
  m_geometry->updateKinematics(updatedModel, Q, Qdot);
}
