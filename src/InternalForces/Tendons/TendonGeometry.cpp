#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/TendonGeometry.h"

#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "Utils/Error.h"


using namespace BIORBD_NAMESPACE;

internal_forces::tendons::TendonGeometry::TendonGeometry()
    : m_originSegmentName(std::make_shared<utils::String>()),
      m_origin(std::make_shared<utils::Vector3d>()),
      m_insertionSegmentName(std::make_shared<utils::String>()),
      m_insertion(std::make_shared<utils::Vector3d>()),
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()) {}

internal_forces::tendons::TendonGeometry::TendonGeometry(
    const utils::String& originSegmentName,
    const utils::Vector3d& origin,
    const utils::String& insertionSegmentName,
    const utils::Vector3d& insertion)
    : m_originSegmentName(std::make_shared<utils::String>(originSegmentName)),
      m_origin(std::make_shared<utils::Vector3d>(origin)),
      m_insertionSegmentName(
          std::make_shared<utils::String>(insertionSegmentName)),
      m_insertion(std::make_shared<utils::Vector3d>(insertion)),
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()) {}

internal_forces::tendons::TendonGeometry
internal_forces::tendons::TendonGeometry::DeepCopy() const {
  internal_forces::tendons::TendonGeometry copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::tendons::TendonGeometry::DeepCopy(
    const TendonGeometry& other) {
  m_originSegmentName = std::make_shared<utils::String>(*other.m_originSegmentName);
  m_origin = std::make_shared<utils::Vector3d>(*other.m_origin);
  m_insertionSegmentName = std::make_shared<utils::String>(*other.m_insertionSegmentName);
  m_insertion = std::make_shared<utils::Vector3d>(*other.m_insertion);

  m_routingPoints->resize(other.m_routingPoints->size());
  for (size_t i = 0; i < other.m_routingPoints->size(); ++i) {
    (*m_routingPoints)[i] = std::make_shared<internal_forces::tendons::TendonRoutingPoint>(
        (*other.m_routingPoints)[i]->DeepCopy());
  }
}

void internal_forces::tendons::TendonGeometry::addRoutingPoint(
  const TendonRoutingPoint& routingPoint) {
  m_routingPoints->push_back(std::make_shared<TendonRoutingPoint>(routingPoint));
}

size_t internal_forces::tendons::TendonGeometry::nbRoutingPoints() const {
  return m_routingPoints->size();
}

internal_forces::tendons::TendonRoutingPoint&
internal_forces::tendons::TendonGeometry::routingPoint(size_t idx) const {
  utils::Error::check(
      idx < m_routingPoints->size(),
      "Idx asked is higher than number of tendon routing points");
  return *(*m_routingPoints)[idx];
}

const utils::Matrix& internal_forces::tendons::TendonGeometry::lengthsJacobian() const {
  return *m_lengthsJacobian;
}

void internal_forces::tendons::TendonGeometry::computeLengthsJacobian() {

}
