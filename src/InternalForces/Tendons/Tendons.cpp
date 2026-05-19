#define BIORBD_API_EXPORTS
#include "InternalForces/Tendons/Tendons.h"

#include "InternalForces/Tendons/Tendon.h"
#include "InternalForces/Tendons/TendonGeometry.h"
#include "Utils/Error.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Joints.h"
#include "Utils/Matrix.h"

using namespace BIORBD_NAMESPACE;

internal_forces::tendons::Tendons::Tendons()
    : m_tendons(
          std::make_shared<std::vector<std::shared_ptr<
              internal_forces::tendons::Tendon>>>()) {}

internal_forces::tendons::Tendons::Tendons(
    const internal_forces::tendons::Tendons& other)
    : m_tendons(other.m_tendons) {}

internal_forces::tendons::Tendons::~Tendons() {}

internal_forces::tendons::Tendons
internal_forces::tendons::Tendons::DeepCopy() const {
  internal_forces::tendons::Tendons copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::tendons::Tendons::DeepCopy(
    const internal_forces::tendons::Tendons& other) {
  m_tendons->resize(other.m_tendons->size());
  for (size_t i = 0; i < other.m_tendons->size(); ++i) {
    (*m_tendons)[i] = std::make_shared<internal_forces::tendons::Tendon>(
        *(*other.m_tendons)[i]);
  }
}

void internal_forces::tendons::Tendons::addTendon(
    const utils::String& name,
    const TendonGeometry& geometry) {
  m_tendons->push_back(
      std::make_shared<internal_forces::tendons::Tendon>(name, geometry));
}

std::vector<std::shared_ptr<internal_forces::tendons::Tendon>>
internal_forces::tendons::Tendons::tendons() const {
  return *m_tendons;
}

internal_forces::tendons::Tendon& internal_forces::tendons::Tendons::tendon(
    size_t idx) const {
  utils::Error::check(
      idx < m_tendons->size(),
      "Idx asked is higher than number of tendons");
  return *(*m_tendons)[idx];
}

std::vector<utils::String>
internal_forces::tendons::Tendons::tendonNames() const {
  std::vector<utils::String> names;
  for (const auto& tendon : *m_tendons) {
    names.push_back(*tendon->m_name);
  }
  return names;
}

size_t internal_forces::tendons::Tendons::nbTendons() const {
  return m_tendons->size();
}

size_t internal_forces::tendons::Tendons::nbTotalTendonSections() const {
  size_t totalSections = 0;
  for (const auto& tendon : *m_tendons) {
    totalSections += 1 + tendon->nbRoutingPoints();
  }
  return totalSections;
}

rigidbody::GeneralizedTorque internal_forces::tendons::Tendons::jointTorquesFromTendons(
    const utils::Vector& tendonForces,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    bool with_friction) {
  utils::Error::check(
      static_cast<size_t>(tendonForces.size()) == nbTendons(),
      "tendonForces size must match the number of tendons");

  // Update the link positions first
  rigidbody::Joints updatedModel = dynamic_cast<rigidbody::Joints&>(*this).UpdateKinematicsCustom(&Q);
  // Update the corresponding positions of origins, routing points, and insertions of all tendons.
  // Further the update computes the tendon length, velocity and the positions and lengths jacobian
  for (auto tendon : *m_tendons) {
    tendon->updateKinematics(updatedModel, Q, Qdot);
  }

  // TODO: tendon; Handle negative pull forces

  if (with_friction) {
    const utils::Matrix& sectionsJaco(tendonSectionLengthsJacobian());
    const utils::Vector& expandedForces(expandTendonPullForcesToSections(tendonForces));
    return rigidbody::GeneralizedTorque(-sectionsJaco.transpose() * expandedForces);
  } else {
    const utils::Matrix& jaco(tendonLengthsJacobian());
    return rigidbody::GeneralizedTorque(-jaco.transpose() * tendonForces);
  }
}

utils::Matrix internal_forces::tendons::Tendons::tendonLengthsJacobian() {
  const auto& model = dynamic_cast<rigidbody::Joints&>(*this);

  // Build the tendon lengths jacobian by stacking the lengths jacobians of all tendons
  utils::Matrix tp(nbTendons(), model.nbDof());
  for (size_t i = 0; i < nbTendons(); ++i) {
    tp.block(i, 0, 1, static_cast<unsigned int>(model.nbDof())) =
      ((*m_tendons)[i])->geometry().lengthsJacobian();
  }
  return tp;
}

utils::Matrix internal_forces::tendons::Tendons::tendonSectionLengthsJacobian() {
  const auto& model = dynamic_cast<rigidbody::Joints&>(*this);

  // Build the tendon lengths jacobian by stacking the lengths jacobians of all tendons
  utils::Matrix tp(nbTotalTendonSections(), model.nbDof());
  size_t rowIdx = 0;
  for (size_t i = 0; i < nbTendons(); ++i) {
    const auto& tendonSectionJaco = (*m_tendons)[i]->geometry().sectionLengthsJacobian();
    tp.block(rowIdx, 0, tendonSectionJaco.rows(), static_cast<unsigned int>(model.nbDof())) =
      tendonSectionJaco;
    rowIdx += tendonSectionJaco.rows();
  }
  return tp;
}

utils::Vector internal_forces::tendons::Tendons::expandTendonPullForcesToSections(
    const utils::Vector& tendonForces) const {
  utils::Vector expandedForces(nbTotalTendonSections());
  expandedForces.setZero();

  size_t sectionIdx = 0;
  for (size_t i = 0; i < m_tendons->size(); ++i) {
    const auto& tendon = *(*m_tendons)[i];
    const auto& frictionLosses = tendon.geometry().sectionFrictionLosses();
    const auto& tendonForce = tendonForces(i);
    utils::Error::check(
        frictionLosses.size() == tendon.nbSections(),
        "Each tendon must have one friction loss per section");

    for (size_t j = 0; j < tendon.nbSections(); ++j) {
      if (j == 0) {
        expandedForces(sectionIdx) = tendonForce * (1-frictionLosses[0]);
      } else {
        // TODO: tendon; Add friction loss of routing points ("wrapping")
        expandedForces(sectionIdx + j) = expandedForces(sectionIdx + j - 1) * (1 - frictionLosses[j]);
      }
    }
    sectionIdx += tendon.nbSections();
  }
  return expandedForces;
}
