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

rigidbody::GeneralizedTorque internal_forces::tendons::Tendons::jointTorquesFromTendons(
    const utils::Vector& tendonForces,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) {
  // Tendon-lengths-jacobian (dL/dq) * -tendon pull forces
  const utils::Matrix& jaco(tendonLengthsJacobian());
  return rigidbody::GeneralizedTorque(-jaco.transpose() * tendonForces);
}

utils::Matrix internal_forces::tendons::Tendons::tendonLengthsJacobian() {
  const rigidbody::Joints& model = dynamic_cast<rigidbody::Joints&>(*this);

  utils::Matrix tp(nbTendons(), model.nbDof());
  for (size_t i = 0; i < nbTendons(); ++i) {
    tp.block(i, 0, 1, static_cast<unsigned int>(model.nbDof())) =
      ((*m_tendons)[i])->geometry().lengthsJacobian();
  }
  return tp;
}
