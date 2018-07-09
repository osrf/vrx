#include "helpers.hh"
#include <gtest/gtest.h>

/// \brief Tests that the basic sandisland world with the wamv was loaded
class SandislandTest : public ::testing::Test
{
protected:
  /// \brief Initialize any members needed for the test cases
  static void SetUpTestCase();
};
void SandislandTest::SetUpTestCase()
{
}

/// \brief Tests that the wamv model exists
TEST_F(SandislandTest, WamvExists)
{
  EXPECT_TRUE(ModelExists("wamv"));
}

/// \brief Tests that the sandisland model exist

TEST_F(SandislandTest, SandislandExists)
{
  EXPECT_TRUE(ModelExists("sandisland"));
}

/// \brief Tests that the ocean model exists
TEST_F(SandislandTest, OceanExists)
{
  EXPECT_TRUE(ModelExists("ocean"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sandisland_test");
  return RUN_ALL_TESTS();
}
