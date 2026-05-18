#ifndef BIORBD_TENDONS_TENDON_GEOMETRY_H
#define BIORBD_TENDONS_TENDON_GEOMETRY_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

#include "InternalForces/Actuators/ActuatorGauss3p.h"
#include "InternalForces/Tendons/TendonRoutingPoint.h"

#include "RigidBody/Joints.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
class Vector3d;
class Matrix;
}

namespace internal_forces {
namespace tendons {

///
/// \brief Class Geometry of the tendon
///
class BIORBD_API TendonGeometry {
public:
  TendonGeometry();

 ///
 /// @brief Construct tendon geometry
 /// @param originSegmentName The name of the segment where the tendon origin
 /// lies
 /// @param origin The position of the tendon origin on the origin segment
 /// @param insertionSegmentName The name of the segment where the tendon
 /// insertion lies
 /// @param insertion The position of the tendon insertion on the insertion segment
  ///
  TendonGeometry(
    const utils::String& originSegmentName, const utils::Vector3d& origin,
    const utils::String& insertionSegmentName, const utils::Vector3d& insertion);

  TendonGeometry DeepCopy() const;

  void DeepCopy(const TendonGeometry& other);

  void addRoutingPoint(const internal_forces::tendons::TendonRoutingPoint& routingPoint);

  size_t nbRoutingPoints() const;

  TendonRoutingPoint& routingPoint(size_t idx) const;

  ///
  /// \brief Returns the tendon lengths jacobian (dL/dq) for this tendon
  /// The resulting matrix has the shape (1xdof)
  /// \return The tendon lengths jacobian (dL/dq) for this tendon only
  const utils::Matrix& lengthsJacobian() const;

  void updateKinematics(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot);

  utils::Scalar& length() const;

  utils::Scalar& velocity() const;

  const std::vector<utils::Vector3d>& pointsInGlobal() const;

protected:
  void computeLengthsJacobian();

  std::shared_ptr<utils::String> m_originSegmentName;
  std::shared_ptr<utils::Vector3d> m_origin;
  std::shared_ptr<utils::String> m_insertionSegmentName;
  std::shared_ptr<utils::Vector3d> m_insertion;
  std::shared_ptr<std::vector<std::shared_ptr<TendonRoutingPoint>>> m_routingPoints; ///< The routing points where the tendon must go through. They are kept and understood in the order of adding.

  std::shared_ptr<utils::Matrix> m_positionsJacobian; ///< The default jacobian matrix (dp/dq) of the positions (forward kinematics) for this tendon. Shape: ((2+n_rp)*3 x dof)
  std::shared_ptr<utils::Matrix> m_lengthsJacobian; ///< The jacobian matrix (dL/dq) of the tendon lengths. Shape: (1 x dof)
  std::shared_ptr<std::vector<utils::Vector3d>> m_pointsInGlobal; ///< Position of origin-, insertion, and routing-points in the global reference coordinate system
  std::shared_ptr<std::vector<utils::Vector3d>> m_pointsInLocal; ///< Position of origin-, insertion, and routing-points in the local reference coordinate system of the segments they belong to

  std::shared_ptr<utils::Scalar> m_length; ///< The length of the tendon
  std::shared_ptr<utils::Scalar> m_velocity; ///< The velocity of the tendon elongation
};
}
}
}

#endif  // BIORBD_TENDONS_TENDON_GEOMETRY_H
