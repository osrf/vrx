#include <gtest/gtest.h>
#include <gazebo/test/ServerFixture.hh>
#include "usv_gazebo_plugins/buoyancy.hpp"

using namespace gazebo;

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


TEST(BuoyancyPluginTest, Cube) {
  auto cube = buoyancy::make_cube(2, 2, 2);
//  for(const auto& v : cube.verts) {
//    std::cout << v << std::endl;
//  }
}

TEST(BuoyancyPluginTest, Cylinder) {
  auto cylinder = buoyancy::make_cylinder(1, 2, 5);
}

TEST(BuoyancyPluginTest, CubeVolume) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cube(2,2,2);

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::computeVolume(cube);
  EXPECT_FLOAT_EQ(volume, 8.0);
  EXPECT_FLOAT_EQ(centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Z(), 0.0);
}

TEST(BuoyancyPluginTest, CylinderVolume) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cylinder(0.5,2,100);

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::computeVolume(cube);
  EXPECT_NEAR(volume, 1.57, 0.01);
  EXPECT_NEAR(centroid.X(), 0.0, 1e-10);
  EXPECT_NEAR(centroid.Y(), 0.0, 1e-10);
  EXPECT_NEAR(centroid.Z(), 0.0, 1e-10);
}

TEST(BuoyancyPluginTest, CubeVolumeNotSubmerged) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cube(1,1,1);

  buoyancy::Plane waterSurface;
  waterSurface.normal = ignition::math::Vector3d{0,0,1};
  waterSurface.offset = 0.f;

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,2}, ignition::math::Quaterniond{1,0,0,0},
      cube, waterSurface);

  EXPECT_FLOAT_EQ(volume, 0.0);
  EXPECT_FLOAT_EQ(centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Z(), 0.0);
}

TEST(BuoyancyPluginTest, CubeVolumeSubmerged) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cube(1,1,1);

  buoyancy::Plane waterSurface;
  waterSurface.normal = ignition::math::Vector3d{0,0,1};
  waterSurface.offset = 0.f;

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,0.0}, ignition::math::Quaterniond{1,0,0,0},
                                                         cube, waterSurface);

  EXPECT_FLOAT_EQ(volume, 0.5);
  EXPECT_NEAR(centroid.X(), 0.0, 1e-15);
  EXPECT_NEAR(centroid.Y(), 0.0, 1e-15);
  EXPECT_FLOAT_EQ(centroid.Z(), -0.125);

  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,-2}, ignition::math::Quaterniond{1,0,0,0},
                                                         cube, waterSurface);

  EXPECT_FLOAT_EQ(volume, 1.0);
  EXPECT_NEAR(centroid.X(), 0.0, 1e-15);
  EXPECT_NEAR(centroid.Y(), 0.0, 1e-15);
//  EXPECT_FLOAT_EQ(centroid.Z(), -0.5);
}

TEST(BuoyancyPluginTest, CylinderVolumeNotSubmerged) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cylinder(0.5,2,10);

  buoyancy::Plane waterSurface;
  waterSurface.normal = ignition::math::Vector3d{0,0,1};
  waterSurface.offset = 0.f;

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,2}, ignition::math::Quaterniond{1,0,0,0},
                                                         cube, waterSurface);

  EXPECT_FLOAT_EQ(volume, 0.0);
  EXPECT_FLOAT_EQ(centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(centroid.Z(), 0.0);
}

TEST(BuoyancyPluginTest, CylinderVolumeSubmerged) {
  // make cube
  buoyancy::Polyhedron cube = buoyancy::make_cylinder(0.5,2,100);

  buoyancy::Plane waterSurface;
  waterSurface.normal = ignition::math::Vector3d{0,0,1};
  waterSurface.offset = 0.f;

  double volume;
  ignition::math::Vector3d centroid;
  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,0.0}, ignition::math::Quaterniond{1,0,0,0},
                                                         cube, waterSurface);

  EXPECT_NEAR(volume, 0.78, 0.01);
  EXPECT_NEAR(centroid.X(), 0.0, 1e-15);
  EXPECT_NEAR(centroid.Y(), 0.0, 1e-15);
//  EXPECT_FLOAT_EQ(centroid.Z(), -0.125);

  std::tie(volume, centroid) = buoyancy::submergedVolume(ignition::math::Vector3d{0,0,-2}, ignition::math::Quaterniond{1,0,0,0},
                                                         cube, waterSurface);

  EXPECT_NEAR(volume, 1.57, 0.01);
  EXPECT_NEAR(centroid.X(), 0.0, 1e-15);
  EXPECT_NEAR(centroid.Y(), 0.0, 1e-15);
//  EXPECT_FLOAT_EQ(centroid.Z(), -0.5);
}


/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}