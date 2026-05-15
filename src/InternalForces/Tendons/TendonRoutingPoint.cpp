#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/TendonRoutingPoint.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::TendonRoutingPoint::TendonRoutingPoint()
    : utils::Vector3d(),
      m_frictionLoss(std::make_shared<utils::Scalar>(1.0)) {
  *m_typeOfNode = utils::NODE_TYPE::TENDON_ROUTING_POINT;
}

internal_forces::tendons::TendonRoutingPoint::TendonRoutingPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String& name,
    const utils::String& parentSegmentName,
    const utils::Scalar& frictionLoss)
    : utils::Vector3d(x, y, z, name, parentSegmentName),
      m_frictionLoss(std::make_shared<utils::Scalar>(frictionLoss)) {
  *m_typeOfNode = utils::NODE_TYPE::TENDON_ROUTING_POINT;
}

internal_forces::tendons::TendonRoutingPoint::TendonRoutingPoint(
    const utils::Vector3d& other)
    : utils::Vector3d(other),
      m_frictionLoss(std::make_shared<utils::Scalar>(1.0)) {
  *m_typeOfNode = utils::NODE_TYPE::TENDON_ROUTING_POINT;
}

internal_forces::tendons::TendonRoutingPoint::TendonRoutingPoint(
    const internal_forces::tendons::TendonRoutingPoint& other)
    : utils::Vector3d(other), m_frictionLoss(other.m_frictionLoss) {}

internal_forces::tendons::TendonRoutingPoint
internal_forces::tendons::TendonRoutingPoint::DeepCopy() const {
  internal_forces::tendons::TendonRoutingPoint copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::tendons::TendonRoutingPoint::DeepCopy(
    const internal_forces::tendons::TendonRoutingPoint& other) {
  utils::Vector3d::DeepCopy(other);
  *m_frictionLoss = *other.m_frictionLoss;
}

const utils::Scalar& internal_forces::tendons::TendonRoutingPoint::frictionLoss()
    const {
  return *m_frictionLoss;
}
