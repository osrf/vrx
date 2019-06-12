#include <gtest/gtest.h>
#include <gazebo/test/ServerFixture.hh>
#include "usv_gazebo_plugins/polyhedron_volume.hpp"

using namespace gazebo;
using namespace buoyancy;

class BuoyancyPluginTest : public ServerFixture {
public:
  BuoyancyPluginTest() {
    Load("test/worlds/buoyancy_plugin_test.world", true);
  }
};

/////////////////////////////////////////////////
//TEST_F(BuoyancyPluginTest, FloatingBox) {
//  physics::WorldPtr world = physics::get_world("buoyancy_test");
//  ASSERT_TRUE(world != nullptr);
//
//  physics::ModelPtr model = world->ModelByName("1_box");
//  ASSERT_TRUE(model != nullptr);
//}

/////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CubeTotalVolume)
{
  auto cube = Polyhedron::makeCube(2,2,2);
  auto volume = cube.computeFullVolume();
  EXPECT_FLOAT_EQ(volume.volume, 8.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

/////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CylinderTotalVolume)
{
  auto cylinder = Polyhedron::makeCylinder(0.5,2,100);
  auto volume = cylinder.computeFullVolume();
  EXPECT_NEAR(volume.volume, 1.57, 0.01);
  EXPECT_NEAR(volume.centroid.X(), 0.0, 1e-10);
  EXPECT_NEAR(volume.centroid.Y(), 0.0, 1e-10);
  EXPECT_NEAR(volume.centroid.Z(), 0.0, 1e-10);
}

///////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CubeNotSubmerged)
{
  auto cube = Polyhedron::makeCube(1,1,1);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  auto volume = cube.submergedVolume(ignition::math::Vector3d{0,0,2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);

  EXPECT_FLOAT_EQ(volume.volume, 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

///////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CylinderNotSubmerged)
{
  auto cylinder = Polyhedron::makeCylinder(0.5,2,10);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  auto volume = cylinder.submergedVolume(ignition::math::Vector3d{0,0,2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);

  EXPECT_FLOAT_EQ(volume.volume, 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

///////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CubeSubmerged)
{
  auto cube = Polyhedron::makeCube(1,1,1);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  // half of the cube is submerged
  //        -----
  // -------|   |--------
  //        -----
  auto volume = cube.submergedVolume(ignition::math::Vector3d{0,0,0},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_FLOAT_EQ(volume.volume, 0.5);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -0.25);

  // cube is fully submerged
  // -------------------
  //        -----
  //        |   |
  //        -----
  volume = cube.submergedVolume(ignition::math::Vector3d{0,0,-2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_FLOAT_EQ(volume.volume, 1.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -2.0);

  // cube is slightly submerged at a 45 degree angle in roll
  //       /\
  //      /  \
  // -----\  /---------
  //       \/
  volume = cube.submergedVolume(ignition::math::Vector3d{0,0,0.25},
      ignition::math::Quaterniond{0.9238795, 0.3826834, 0, 0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.21, 0.01);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_NEAR(volume.centroid.Z(), -0.15, 0.01);
}

///////////////////////////////////////////////////
TEST(BuoyancyPluginTest, CylinderSubmerged)
{
  auto cylinder = Polyhedron::makeCylinder(0.5, 2, 100);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  // half of the cylinder is submerged
  //        ---
  // -------| |--------
  //        ---
  auto volume = cylinder.submergedVolume(ignition::math::Vector3d{0,0,0.0},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.785, 0.001);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -0.5);

  // cylinder is fully submerged
  // -------------------
  //        ---
  //        | |
  //        ---
  volume = cylinder.submergedVolume(ignition::math::Vector3d{0,0,-4},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 1.57, 0.01);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -4.0);

  // cube is half submerged at a 45 degree angle in roll
  //     --------
  // ----|      |------
  //     --------
  volume = cylinder.submergedVolume(ignition::math::Vector3d{0,0,0},
      ignition::math::Quaterniond{0.707,0.707,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.785, 0.001);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_NEAR(volume.centroid.Z(), -0.21, 0.01);
}


/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}