#include "biorbdConfig.h"

#include <gtest/gtest.h>

#include "BiorbdModel.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Utils/Vector.h"
#include "Utils/String.h"
#include "InternalForces/Tendons/all.h"

using namespace BIORBD_NAMESPACE;

TEST(Tendons, parseFromModelFile) {
  Model model("models/two_segments_with_tendon.bioMod");
  EXPECT_EQ(model.nbTendons(), 1);
  EXPECT_EQ(model.tendonNames().size(), 1);
  EXPECT_STREQ(model.tendonNames()[0].c_str(), "t");
}

TEST(Tendons, parseRoutingPointFromModelFile) {
  Model model("models/tendon_finger.bioMod");
  EXPECT_EQ(model.nbTendons(), 1);
  EXPECT_EQ(model.tendon(0).nbRoutingPoints(), 2);
  EXPECT_STREQ(model.tendon(0).routingPoint(0).parent().c_str(), "proximal");
  SCALAR_TO_DOUBLE(y, model.tendon(0).routingPoint(0).y());
  EXPECT_DOUBLE_EQ(y, 0.5);
  SCALAR_TO_DOUBLE(floss, model.tendon(0).routingPoint(0).frictionLoss())
  EXPECT_DOUBLE_EQ(floss, 0.1);
}

TEST(Tendons, jointTorquesFromTendons) {
  Model model("models/two_segments_with_tendon.bioMod");
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);

  utils::Vector tendonForces(model.nbTendons());
  tendonForces.setZero();

  {
    const auto tau = model.jointTorquesFromTendons(tendonForces, Q, Qdot);
    EXPECT_EQ(static_cast<size_t>(tau.size()), model.nbDof());
    for (unsigned int i = 0; i < tau.size(); ++i) {
      // Flat configuration => no lever arm => no joint torque
      SCALAR_TO_DOUBLE(flatTauNonForce, tau(i));
      EXPECT_NEAR(flatTauNonForce, 0.0, 1e-5);
    }

    // In the flat configuration, the tendons length is exactly one.
    // Since there is no lever arm in the flat configuration, the velocity is 0
    SCALAR_TO_DOUBLE(tendonLength, model.tendons()[0]->geometry().length());
    EXPECT_NEAR(tendonLength, 1.0, 1e-5);
    SCALAR_TO_DOUBLE(tendonVelocity, model.tendons()[0]->geometry().velocity());
    EXPECT_NEAR(tendonVelocity, 0.0, 1e-5);
  }

  {
    tendonForces = casadi::DM::vertcat({400});
    const auto tau = model.jointTorquesFromTendons(tendonForces, Q, Qdot);
    EXPECT_EQ(static_cast<size_t>(tau.size()), model.nbDof());
    for (unsigned int i = 0; i < tau.size(); ++i) {
      // Even with pull forces, there can be no motion (no lever arm!)
      SCALAR_TO_DOUBLE(flatTauForce, tau(i));
      EXPECT_NEAR(flatTauForce, 0.0, 1e-5);
    }
  }

  {
    Q = casadi::DM::vertcat({M_PI / 2});
    Qdot = casadi::DM::vertcat({0.0});
    tendonForces = casadi::DM::vertcat({10});
    // When the tendon angle is 45°, the x component of the force is 10*cos(0.5)=7.07
    const double tendonForceX = 10*cos(M_PI/4);
    // The lever arm is 0.5
    const double expectedJointTorque = tendonForceX*0.5;

    const auto tau = model.jointTorquesFromTendons(tendonForces, Q, Qdot);
    EXPECT_EQ(static_cast<size_t>(tau.size()), model.nbDof());
    for (unsigned int i = 0; i < tau.size(); ++i) {
      SCALAR_TO_DOUBLE(tauForce, tau(i));
      EXPECT_NEAR(tauForce, expectedJointTorque, 1e-5);
    }

    SCALAR_TO_DOUBLE(tendonLength, model.tendons()[0]->geometry().length());
    EXPECT_NEAR(tendonLength, sqrt(0.5), 1e-5);
    SCALAR_TO_DOUBLE(tendonVelocity, model.tendons()[0]->geometry().velocity());
    EXPECT_NEAR(tendonVelocity, 0.0, 1e-5);
  }
}

TEST(Tendons, jointTorquesFromRoutingPointTendons) {
  Model model("models/tendon_finger.bioMod");
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);

  utils::Vector tendonForces(model.nbTendons());
  tendonForces.setZero();

  {
    const auto tau = model.jointTorquesFromTendons(tendonForces, Q, Qdot);
    EXPECT_EQ(static_cast<size_t>(tau.size()), model.nbDof());
    for (unsigned int i = 0; i < tau.size(); ++i) {
      // Flat configuration => no lever arm => no joint torque
      SCALAR_TO_DOUBLE(flatTauNonForce, tau(i));
      EXPECT_NEAR(flatTauNonForce, 0.0, 1e-5);
    }

    // In the flat configuration, the tendons length is exactly one.
    // Since there is no lever arm in the flat configuration, the velocity is 0
    SCALAR_TO_DOUBLE(tendonLength, model.tendons()[0]->geometry().length());
    EXPECT_NEAR(tendonLength, 2.7, 1e-5);
    SCALAR_TO_DOUBLE(tendonVelocity, model.tendons()[0]->geometry().velocity());
    EXPECT_NEAR(tendonVelocity, 0.0, 1e-5);
  }
}
