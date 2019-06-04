#include <gtest/gtest.h>
#include <gazebo/test/ServerFixture.hh>

using namespace gazebo;

class BuoyancyPluginTest : public ServerFixture {
public:
  BuoyancyPluginTest() {
    Load("test/worlds/buoyancy_plugin_test.world", true);
  }
};

/////////////////////////////////////////////////
TEST_F(BuoyancyPluginTest, FloatingBox) {
  physics::WorldPtr world = physics::get_world("buoyancy_test");
  ASSERT_TRUE(world != nullptr);

  physics::ModelPtr model = world->ModelByName("1_box");
  ASSERT_TRUE(model != nullptr);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}