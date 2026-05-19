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
      m_sectionFrictionLosses(std::make_shared<std::vector<utils::Scalar>>(std::initializer_list<utils::Scalar>{utils::Scalar(0)})),
      m_positionsJacobian(std::make_shared<utils::Matrix>()),
      m_lengthsJacobian(std::make_shared<utils::Matrix>()),
      m_sectionLengthsJacobian(std::make_shared<utils::Matrix>()),
      m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_length(std::make_shared<utils::Scalar>(0)),
      m_sectionLengths(std::make_shared<utils::Vector>()),
      m_velocity(std::make_shared<utils::Scalar>(0)),
      m_sectionVelocities(std::make_shared<utils::Vector>()) {}

internal_forces::tendons::TendonGeometry::TendonGeometry(
    const utils::String& originSegmentName,
    const utils::Vector3d& origin,
    const utils::String& insertionSegmentName,
    const utils::Vector3d& insertion,
    const utils::Scalar& firstSectionFrictionLoss)
    : m_originSegmentName(std::make_shared<utils::String>(originSegmentName)),
      m_origin(std::make_shared<utils::Vector3d>(origin)),
      m_insertionSegmentName(std::make_shared<utils::String>(insertionSegmentName)),
      m_insertion(std::make_shared<utils::Vector3d>(insertion)),
      m_routingPoints(std::make_shared<std::vector<std::shared_ptr<internal_forces::tendons::TendonRoutingPoint>>>()),
      m_sectionFrictionLosses(std::make_shared<std::vector<utils::Scalar>>(std::initializer_list<utils::Scalar>{firstSectionFrictionLoss})),
      m_positionsJacobian(std::make_shared<utils::Matrix>()),
      m_sectionLengthsJacobian(std::make_shared<utils::Matrix>()),
      m_lengthsJacobian(std::make_shared<utils::Matrix>()),
      m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
      m_length(std::make_shared<utils::Scalar>(0)),
      m_sectionLengths(std::make_shared<utils::Vector>()),
      m_velocity(std::make_shared<utils::Scalar>(0)),
      m_sectionVelocities(std::make_shared<utils::Vector>()) {}

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

  m_sectionFrictionLosses->resize(other.m_sectionFrictionLosses->size());
  for (size_t i = 0; i < other.m_sectionFrictionLosses->size(); ++i) {
    (*m_sectionFrictionLosses)[i] = (*other.m_sectionFrictionLosses)[i];
  }

  *m_positionsJacobian = *other.m_positionsJacobian;
  *m_lengthsJacobian = *other.m_lengthsJacobian;
  *m_sectionLengthsJacobian = *other.m_sectionLengthsJacobian;
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
  *m_sectionLengths = *other.m_sectionLengths;
  *m_sectionVelocities = *other.m_sectionVelocities;
}

void internal_forces::tendons::TendonGeometry::addRoutingPoint(
  const TendonRoutingPoint& routingPoint,
  const utils::Scalar& sectionFrictionLoss) {
  m_routingPoints->push_back(std::make_shared<TendonRoutingPoint>(routingPoint));
  m_sectionFrictionLosses->push_back(sectionFrictionLoss);
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

const utils::Matrix& internal_forces::tendons::TendonGeometry::sectionLengthsJacobian() const {
  return *m_sectionLengthsJacobian;
}

void internal_forces::tendons::TendonGeometry::computeLengthsJacobian() const {
  *m_lengthsJacobian = utils::Matrix::Zero(1, m_positionsJacobian->cols());

  for (size_t i = 0; i < m_pointsInGlobal->size()-1; ++i) {
    const auto& p0 = (*m_pointsInGlobal)[i];
    const auto& p1 = (*m_pointsInGlobal)[i+1];
    const auto& posJac0 = m_positionsJacobian->block(3*static_cast<unsigned int>(i), 0, 3, m_positionsJacobian->cols());
    const auto& posJac1 = m_positionsJacobian->block(3*static_cast<unsigned int>(i+1), 0, 3, m_positionsJacobian->cols());
    *m_lengthsJacobian += ((p1-p0).transpose() * (posJac1 - posJac0)) / (p1-p0).norm();
  }
}

void internal_forces::tendons::TendonGeometry::computeSectionLengthsJacobian() const {
  *m_sectionLengthsJacobian = utils::Matrix::Zero(1+nbRoutingPoints(), m_positionsJacobian->cols());
  for (size_t i = 0; i < m_pointsInGlobal->size()-1; ++i) {
    const auto& p0 = (*m_pointsInGlobal)[i];
    const auto& p1 = (*m_pointsInGlobal)[i+1];
    const auto& posJac0 = m_positionsJacobian->block(3*static_cast<unsigned int>(i), 0, 3, m_positionsJacobian->cols());
    const auto& posJac1 = m_positionsJacobian->block(3*static_cast<unsigned int>(i+1), 0, 3, m_positionsJacobian->cols());
    m_sectionLengthsJacobian->block(static_cast<unsigned int>(i), 0, 1, m_positionsJacobian->cols())
      = ((p1-p0).transpose() * (posJac1 - posJac0)) / (p1-p0).norm();
  }
}

void internal_forces::tendons::TendonGeometry::updateKinematics(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) const {
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

  // Compute the positions jacobian for this tendon
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
  *m_sectionLengths = utils::Vector::Zero(static_cast<unsigned int>(m_pointsInGlobal->size()-1));
  for (size_t i = 0; i < m_pointsInGlobal->size()-1; ++i) {
    (*m_sectionLengths)(i) = ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
    *m_length += (*m_sectionLengths)(i);
  }

  // Compute the lengths and section-lengths jacobian
  computeLengthsJacobian();
  computeSectionLengthsJacobian();

  // Compute the tendon velocity. Since the lengths jacobian is needed therefore,
  // it is computed afterward
  *m_velocity = (lengthsJacobian() * Qdot)[0];
  *m_sectionVelocities = (sectionLengthsJacobian() * Qdot);
}

utils::Scalar& internal_forces::tendons::TendonGeometry::length() const {
  return *m_length;
}

utils::Vector& internal_forces::tendons::TendonGeometry::sectionLengths() const {
  return *m_sectionLengths;
}

utils::Scalar& internal_forces::tendons::TendonGeometry::velocity() const {
  return *m_velocity;
}

utils::Vector& internal_forces::tendons::TendonGeometry::sectionVelocities() const {
  return *m_sectionVelocities;
}

const std::vector<utils::Vector3d>& internal_forces::tendons::TendonGeometry::pointsInGlobal() const {
  return *m_pointsInGlobal;
}

const std::vector<utils::Scalar>& internal_forces::tendons::TendonGeometry::sectionFrictionLosses() const {
  return *m_sectionFrictionLosses;
}
