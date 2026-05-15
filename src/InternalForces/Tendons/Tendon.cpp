#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/Tendon.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::Tendon::Tendon()
    : m_name(std::make_shared<utils::String>()),
      m_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>()),
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()) {}

internal_forces::tendons::Tendon::Tendon(
    const utils::String& name,
    const TendonGeometry& geometry)
    : m_name(std::make_shared<utils::String>(name)),
      m_geometry(std::make_shared<internal_forces::tendons::TendonGeometry>(geometry)),
        m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()) {}

internal_forces::tendons::Tendon::Tendon(const Tendon& other)
    : m_name(other.m_name),
      m_geometry(other.m_geometry),
      m_routingPoints(other.m_routingPoints) {}

internal_forces::tendons::Tendon::Tendon(
    const std::shared_ptr<Tendon> other)
    : m_name(other->m_name),
      m_geometry(other->m_geometry),
      m_routingPoints(other->m_routingPoints) {}

void internal_forces::tendons::Tendon::DeepCopy(
    const Tendon& other) {
  m_name = std::make_shared<utils::String>(*other.m_name);
  m_geometry = std::make_shared<internal_forces::tendons::TendonGeometry>(
      other.m_geometry->DeepCopy());
  m_routingPoints->resize(other.m_routingPoints->size());
  for (size_t i = 0; i < other.m_routingPoints->size(); ++i) {
    (*m_routingPoints)[i] = std::make_shared<internal_forces::tendons::TendonRoutingPoint>(
        (*other.m_routingPoints)[i]->DeepCopy());
  }
}

void internal_forces::tendons::Tendon::addRoutingPoint(
  const TendonRoutingPoint& routingPoint) {
  m_routingPoints->push_back(std::make_shared<TendonRoutingPoint>(routingPoint));
}

size_t internal_forces::tendons::Tendon::nbRoutingPoints() const {
  return m_routingPoints->size();
}

internal_forces::tendons::TendonRoutingPoint&
internal_forces::tendons::Tendon::routingPoint(size_t idx) const {
  utils::Error::check(
      idx < m_routingPoints->size(),
      "Idx asked is higher than number of tendon routing points");
  return *(*m_routingPoints)[idx];
}
