#include "biorbdConfig.h"

#include <gtest/gtest.h>

#include "BiorbdModel.h"

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
  EXPECT_STREQ(model.tendon(0).routingPoint(0).name().c_str(), "trp1");
  EXPECT_STREQ(model.tendon(0).routingPoint(0).parent().c_str(), "segment1");
  EXPECT_DOUBLE_EQ(model.tendon(0).routingPoint(0).operator()(1), 0.5);
  EXPECT_DOUBLE_EQ(model.tendon(0).routingPoint(0).frictionLoss(), 0.1);
}
