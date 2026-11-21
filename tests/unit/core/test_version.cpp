#include <algorithm>
#include <string_view>

#include <gtest/gtest.h>

#include "roboticscore/core/version.hpp"

namespace roboticscore {
namespace {

TEST(Version, HeaderAndLibraryAgree) {
  EXPECT_EQ(version(), kVersionString);
}

TEST(Version, IsSemanticVersion) {
  const std::string_view v = version();
  EXPECT_EQ(std::count(v.begin(), v.end(), '.'), 2);
}

}  // namespace
}  // namespace roboticscore
