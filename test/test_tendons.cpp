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
