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
  Model model("models/two_segments_with_tendon_routing_point.bioMod");
  EXPECT_EQ(model.nbTendons(), 1);
  EXPECT_EQ(model.tendon(0).nbRoutingPoints(), 1);
  EXPECT_STREQ(model.tendon(0).routingPoint(0).parent().c_str(), "segment1");
  EXPECT_DOUBLE_EQ(
      static_cast<double>(model.tendon(0).routingPoint(0).operator()(1)), 0.5);
  SCALAR_TO_DOUBLE(floss, model.tendon(0).routingPoint(0).frictionLoss())
  EXPECT_DOUBLE_EQ(floss, 0.1);
}

TEST(Tendons, jointTorquesFromTendons) {
  Model model("models/two_segments_with_tendon.bioMod");
  rigidbody::GeneralizedCoordinates Q(model);
  rigidbody::GeneralizedVelocity Qdot(model);
  Q.setZero();
  Qdot.setZero();

  utils::Vector tendonForces(model.nbTendons());
  tendonForces.setZero();

  const auto torques = model.jointTorquesFromTendons(tendonForces, Q, Qdot);
  EXPECT_EQ(static_cast<size_t>(torques.size()), model.nbDof());
}
