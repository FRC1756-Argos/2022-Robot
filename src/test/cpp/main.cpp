/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <hal/HAL.h>

#include "argos_lib/general/Framer.h"
#include "gtest/gtest.h"
#include "units/angle.h"

namespace CCW {
  Framer::RefFrame frame_360{360_deg};
  Framer::RefFrame frame_270{270_deg};
  Framer::RefFrame frame_180{180_deg};
  Framer::RefFrame frame_90{90_deg};
  Framer::RefFrame frame_0{0_deg};
  Framer::RefFrame frame_Q1Rand{52_deg};
  Framer::RefFrame frame_Q2Rand{135_deg};
  Framer::RefFrame frame_Q3Rand{234_deg};
  Framer::RefFrame frame_Q4Rand{292_deg};
}  // namespace CCW
namespace CW {
  Framer::RefFrame frame_360{-360_deg};
  Framer::RefFrame frame_270{-270_deg};
  Framer::RefFrame frame_180{-180_deg};
  Framer::RefFrame frame_90{-90_deg};
  Framer::RefFrame frame_Q1Rand{-58_deg};
  Framer::RefFrame frame_Q2Rand{-133_deg};
  Framer::RefFrame frame_Q3Rand{-256_deg};
  Framer::RefFrame frame_Q4Rand{-302_deg};
}  // namespace CW

TEST(RefFrameConstructionTest, RootOffsetHandlesPositiveAngles) {
  EXPECT_EQ(CCW::frame_90.GetRootOffset().convert<units::degree>(), 90_deg);
  EXPECT_EQ(CCW::frame_180.GetRootOffset().convert<units::degree>(), 180_deg);
  EXPECT_EQ(CCW::frame_360.GetRootOffset().convert<units::degree>(), 360_deg);
  EXPECT_EQ(CCW::frame_Q1Rand.GetRootOffset().convert<units::degree>(), 52_deg);
  EXPECT_EQ(CCW::frame_Q2Rand.GetRootOffset().convert<units::degree>(), 135_deg);
  EXPECT_EQ(CCW::frame_Q3Rand.GetRootOffset().convert<units::degree>(), 234_deg);
  EXPECT_EQ(CCW::frame_Q4Rand.GetRootOffset().convert<units::degree>(), 292_deg);
}

TEST(RefFrameConstructionTest, RootOffsetHandlesZero) {
  EXPECT_EQ(CCW::frame_0.GetRootOffset().convert<units::degree>(), 360_deg);
}

TEST(RefFrameConstructionTest, RootOffsetHandlesNegativeAngles) {
  EXPECT_EQ(CW::frame_90.GetRootOffset().convert<units::degree>(), -90_deg);
  EXPECT_EQ(CW::frame_180.GetRootOffset().convert<units::degree>(), -180_deg);
  EXPECT_EQ(CW::frame_360.GetRootOffset().convert<units::degree>(), -360_deg);
  EXPECT_EQ(CW::frame_Q1Rand.GetRootOffset().convert<units::degree>(), -58_deg);
  EXPECT_EQ(CW::frame_Q2Rand.GetRootOffset().convert<units::degree>(), -133_deg);
  EXPECT_EQ(CW::frame_Q3Rand.GetRootOffset().convert<units::degree>(), -256_deg);
  EXPECT_EQ(CW::frame_Q4Rand.GetRootOffset().convert<units::degree>(), -302_deg);
}

TEST(FrameAngleConversionTest, RelativeCcwAngles) {
  EXPECT_EQ(CCW::frame_90.ConvertAngle(CCW::frame_0, 0_deg), 90_deg);
  EXPECT_EQ(CCW::frame_180.ConvertAngle(CCW::frame_90, 0_deg), 90_deg);
  EXPECT_EQ(CCW::frame_270.ConvertAngle(CCW::frame_180, 0_deg), 90_deg);
  EXPECT_EQ(CCW::frame_Q1Rand.ConvertAngle(CCW::frame_Q3Rand, 18_deg), 196_deg);
  EXPECT_EQ(CCW::frame_Q4Rand.ConvertAngle(CCW::frame_Q1Rand, 121_deg), 1_deg);
}

TEST(FrameAngleConversionTest, RelativeCwAngles) {
  EXPECT_EQ(CW::frame_360.ConvertAngle(CW::frame_90, 91_deg), 1_deg);
  EXPECT_EQ(CW::frame_90.ConvertAngle(CW::frame_180, 181_deg), 91_deg);
  EXPECT_EQ(CW::frame_180.ConvertAngle(CW::frame_270, 271_deg), 181_deg);
  EXPECT_EQ(CW::frame_Q1Rand.ConvertAngle(CW::frame_Q3Rand, 27_deg), 189_deg);
  EXPECT_EQ(CW::frame_Q4Rand.ConvertAngle(CW::frame_Q1Rand, 48_deg), 292_deg);
}

TEST(FrameAngleConversionTest, RelativeCcwToCwAngles) {
  EXPECT_EQ(CCW::frame_90.ConvertAngle(CW::frame_90, 0_deg), 180_deg);
  EXPECT_EQ(CCW::frame_180.ConvertAngle(CW::frame_180, 90_deg), 270_deg);
  EXPECT_EQ(CCW::frame_270.ConvertAngle(CW::frame_270, 90_deg), 90_deg);
  EXPECT_EQ(CCW::frame_Q2Rand.ConvertAngle(CW::frame_Q3Rand, 82_deg), 247_deg);
  EXPECT_EQ(CCW::frame_Q4Rand.ConvertAngle(CW::frame_Q1Rand, 159_deg), 211_deg);
}

TEST(FrameAngleConversionTest, RelativeCwToCcwAngles) {
  EXPECT_EQ(CW::frame_90.ConvertAngle(CCW::frame_90, 0_deg), 180_deg);
  EXPECT_EQ(CW::frame_180.ConvertAngle(CCW::frame_270, 5_deg), 265_deg);
  EXPECT_EQ(CW::frame_270.ConvertAngle(CCW::frame_0, 0_deg), 90_deg);
  EXPECT_EQ(CW::frame_Q1Rand.ConvertAngle(CCW::frame_Q1Rand, 69_deg), 181_deg);
  EXPECT_EQ(CW::frame_Q4Rand.ConvertAngle(CCW::frame_Q2Rand, 27_deg), 256_deg);
}

TEST(FrameAngleConversionTest, ConstrainsAngleInputs) {
  EXPECT_EQ(CCW::frame_90.ConvertAngle(CCW::frame_0, 720_deg), 90_deg);
  EXPECT_EQ(CW::frame_Q1Rand.ConvertAngle(CW::frame_Q3Rand, 747_deg), 189_deg);
  EXPECT_EQ(CCW::frame_Q4Rand.ConvertAngle(CW::frame_Q1Rand, 1959_deg), 211_deg);
  EXPECT_EQ(CW::frame_Q1Rand.ConvertAngle(CCW::frame_Q1Rand, 1149_deg), 181_deg);
}

TEST(FramePointConversionTest, CorrectRelativePoint) {
  // CCW to CCW
  EXPECT_EQ(CCW::frame_90.ConvertPoint(CCW::frame_0, frc::Translation2d(1_m, 0_m)), frc::Translation2d(0_m, 1_m));
  // CCW to CW
  EXPECT_EQ(CCW::frame_90.ConvertPoint(CW::frame_360, frc::Translation2d(1_m, 0_m)), frc::Translation2d(0_m, 1_m));
  // CW to CCW
  EXPECT_EQ(CW::frame_180.ConvertPoint(CCW::frame_90, frc::Translation2d(1_m, 0_m)), frc::Translation2d(0_m, 1_m));
  // CW to CW
  EXPECT_EQ(CW::frame_90.ConvertPoint(CW::frame_360, frc::Translation2d(1_m, 0_m)), frc::Translation2d(0_m, -1_m));
}

TEST(FrameHomingTest, CorrectCalculatedHomes) {
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(0_deg, 0_deg), 0_deg);
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(68_deg, -90_deg), -202_deg);
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(68_deg, 90_deg), -22_deg);
}

TEST(FrameHomingTest, CorrectInitializedHomes) {
  EXPECT_EQ(Framer::FoRConverter::InitHome(99_deg, -202_deg), 301_deg);
  EXPECT_EQ(Framer::FoRConverter::InitHome(26_deg, -22_deg), 48_deg);
  EXPECT_EQ(Framer::FoRConverter::InitHome(360_deg, -22_deg), 22_deg);

  // Work the same in -180 - 180 space?
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(0_deg, 90_deg), -90_deg);
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(-180_deg, 90_deg), -270_deg);
  EXPECT_EQ(Framer::FoRConverter::CalculateHome(180_deg, 90_deg), 90_deg);

  // Init -180 - 180 homes
  EXPECT_EQ(Framer::FoRConverter::InitHome(178_deg, 270_deg), 268_deg);
  EXPECT_EQ(Framer::FoRConverter::InitHome(0_deg, -270_deg).value(), 270);
  EXPECT_EQ(Framer::FoRConverter::InitHome(-102_deg, 90_deg).value(), 168);

  // If the frame is also of type 0-360, it works fine
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
