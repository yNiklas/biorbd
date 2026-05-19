#ifndef BIORBD_TENDONS_H
#define BIORBD_TENDONS_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

#include "InternalForces/Actuators/ActuatorGauss3p.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
class Vector;
class Matrix;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace internal_forces {
namespace tendons {
class Tendon;
class TendonGeometry;

class BIORBD_API Tendons {
public:
  Tendons();

  Tendons(const Tendons& other);

  virtual ~Tendons();

  Tendons DeepCopy() const;

  void DeepCopy(const Tendons& other);

  void addTendon(
      const utils::String& name,
      const TendonGeometry& geometry);

  std::vector<std::shared_ptr<Tendon>> tendons() const;

  Tendon& tendon(size_t idx) const;

  std::vector<utils::String> tendonNames() const;

  size_t nbTendons() const;

  size_t nbTotalTendonSections() const;

  ///
  /// \brief Compute the joint torques from tendon actuations
  /// \param tendonForces The pull forces vector of all the tendons. Shape: (nbTendons)
  /// \param Q The generalized coordinates
  /// \param Qdot The generalized velocities
  /// \param with_friction Whether the tendon forces should be expanded to
  ///        sections and apply the specified friction losses: (nbTendons) -> (nbTendons * n_sections_per_tendon) = (nbTendons) ->  (nbTotalTendonSections)
  /// \return The generalized joint torques, based on the actuated tendons
  ///
  rigidbody::GeneralizedTorque jointTorquesFromTendons(
      const utils::Vector& tendonForces,
      const rigidbody::GeneralizedCoordinates& Q,
      const rigidbody::GeneralizedVelocity& Qdot,
      bool with_friction = true);

  ///
  /// \brief Constructs the jacobian matrix (dL/dq) of all tendon lengths.
  ///        The resulting matrix has the shape (nbTendons x nb_q).
  /// \return The jacobian (dL/dq) of the tendon lengths
  ///
  utils::Matrix tendonLengthsJacobian();

  ///
  /// \brief Constructs the jacobian matrix (dLs/dq) of all tendons sections of all tendons.
  ///        The resulting matrix has the shape (nbTendons * n_sections_per_tendon x nb_q) = (nbTotalTendonSections).
  /// \return The jacobian (dLs/dq) of the tendon sections lengths
  ///
  utils::Matrix tendonSectionLengthsJacobian();

protected:
  utils::Vector expandTendonPullForcesToSections(const utils::Vector& tendonForces) const;

  std::shared_ptr<std::vector<std::shared_ptr<Tendon>>> m_tendons;
};
}
}
}

#endif  // BIORBD_TENDONS_H
