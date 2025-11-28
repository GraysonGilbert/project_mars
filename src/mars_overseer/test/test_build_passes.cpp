#include <gtest/gtest.h>

// Simple test to ensure that the test framework is working
TEST(SanityCheck, BuildPasses) { EXPECT_EQ(1, 1); }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
