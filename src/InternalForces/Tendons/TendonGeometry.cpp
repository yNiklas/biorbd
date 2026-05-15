#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/TendonGeometry.h"

#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "RigidBody/GeneralizedVelocity.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::TendonGeometry::TendonGeometry()
    : m_originSegmentName(std::make_shared<utils::String>()),
      m_origin(std::make_shared<utils::Vector3d>()),
      m_insertionSegmentName(std::make_shared<utils::String>()),
      m_insertion(std::make_shared<utils::Vector3d>()),
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()),
      m_positionsJacobian(std::make_shared<utils::Matrix>()),
      m_lengthsJacobian(std::make_shared<utils::Matrix>()),
      m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_length(std::make_shared<utils::Scalar>(0)),
      m_velocity(std::make_shared<utils::Scalar>(0)) {}

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
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()),
      m_positionsJacobian(std::make_shared<utils::Matrix>()),
      m_lengthsJacobian(std::make_shared<utils::Matrix>()),
      m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_length(std::make_shared<utils::Scalar>(0)),
      m_velocity(std::make_shared<utils::Scalar>(0)) {}

internal_forces::tendons::TendonGeometry
internal_forces::tendons::TendonGeometry::DeepCopy() const {
  internal_forces::tendons::TendonGeometry copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::tendons::TendonGeometry::DeepCopy(
    const TendonGeometry& other) {
  *m_originSegmentName = *other.m_originSegmentName;
  *m_origin = other.m_origin->DeepCopy();
  *m_insertionSegmentName = *other.m_insertionSegmentName;
  *m_insertion = other.m_insertion->DeepCopy();

  m_routingPoints->resize(other.m_routingPoints->size());
  for (size_t i = 0; i < other.m_routingPoints->size(); ++i) {
    (*m_routingPoints)[i] = std::make_shared<internal_forces::tendons::TendonRoutingPoint>(
        (*other.m_routingPoints)[i]->DeepCopy());
  }

  *m_positionsJacobian = *other.m_positionsJacobian;
  *m_lengthsJacobian = *other.m_lengthsJacobian;
  m_pointsInGlobal->resize(other.m_pointsInGlobal->size());
  for (size_t i = 0; i < other.m_pointsInGlobal->size(); ++i) {
    (*m_pointsInGlobal)[i] = (*other.m_pointsInGlobal)[i].DeepCopy();
  }
  m_pointsInLocal->resize(other.m_pointsInLocal->size());
  for (size_t i = 0; i < other.m_pointsInLocal->size(); ++i) {
    (*m_pointsInLocal)[i] = (*other.m_pointsInLocal)[i].DeepCopy();
  }
  *m_length = *other.m_length;
  *m_velocity = *other.m_velocity;
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
  *m_lengthsJacobian = utils::Matrix::Zero(1, m_positionsJacobian->cols());

  for (size_t i = 0; i < m_pointsInGlobal->size()-1; ++i) {
    const auto& p0 = (*m_pointsInGlobal)[i];
    const auto& p1 = (*m_pointsInGlobal)[i+1];
    const auto& posJac0 = m_positionsJacobian->block(3*static_cast<unsigned int>(i), 0, 3, m_positionsJacobian->cols());
    const auto& posJac1 = m_positionsJacobian->block(3*static_cast<unsigned int>(i+1), 0, 3, m_positionsJacobian->cols());
    *m_lengthsJacobian += ((p1-p0).transpose() * (posJac1 - posJac0)) / (p1-p0).norm();
  }
}

void internal_forces::tendons::TendonGeometry::updateKinematics(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) {
  m_pointsInLocal->clear();
  m_pointsInGlobal->clear();

  // Add the origin point
  m_pointsInLocal->push_back(*m_origin);
  m_pointsInGlobal->push_back(updatedModel.CalcBodyToBaseCoordinates(
    Q,
    *m_originSegmentName,
    *m_origin,
    false));

  // Add the routing points
  for (const auto& routingPoint : *m_routingPoints) {
    m_pointsInLocal->push_back(*routingPoint);
    m_pointsInGlobal->push_back(updatedModel.CalcBodyToBaseCoordinates(
      Q,
      routingPoint->parent(),
      *routingPoint,
      false));
  }

  // Add the insertion point
  m_pointsInLocal->push_back(*m_insertion);
  m_pointsInGlobal->push_back(updatedModel.CalcBodyToBaseCoordinates(
    Q,
    *m_insertionSegmentName,
    *m_insertion,
    false));

  // Compute the positions jacobian
  *m_positionsJacobian = utils::Matrix::Zero(
    static_cast<unsigned int>(m_pointsInLocal->size()*3),
    updatedModel.dof_count);

  for (size_t i = 0; i < m_pointsInLocal->size(); ++i) {
    utils::String parentName;
    if (i == 0) {
      parentName = *m_originSegmentName;
    } else if (i + 1 == m_pointsInLocal->size()) {
      parentName = *m_insertionSegmentName;
    } else {
      parentName = (*m_pointsInLocal)[i].parent();
    }
    m_positionsJacobian->block(3*static_cast<unsigned int>(i), 0, 3, updatedModel.dof_count)
      = updatedModel.CalcPointJacobian(Q, parentName, (*m_pointsInLocal)[i], false);
  }

  // Compute the tendon length
  *m_length = 0;
  for (size_t i = 0; i < m_pointsInGlobal->size()-1; ++i) {
    *m_length += ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
  }

  // Compute the lengths jacobian
  computeLengthsJacobian();

  // Compute the tendon velocity. Since the lengths jacobian is needed therefore,
  // it is computed afterward
  *m_velocity = (lengthsJacobian() * Qdot)[0];
}
