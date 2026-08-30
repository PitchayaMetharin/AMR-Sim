#include <chrono>

#include <gtest/gtest.h>

#include "amr_simulation/command_watchdog.hpp"

using namespace std::chrono_literals;

TEST(CommandWatchdog, StartsDisabledAndExpiresFreshCommand) {
  amr_simulation::CommandWatchdog watchdog(200ms);
  EXPECT_EQ(watchdog.update(0ms, false, false), false);
  EXPECT_EQ(watchdog.update(10ms, true, false), true);
  EXPECT_FALSE(watchdog.update(210ms, false, false).has_value());
  EXPECT_EQ(watchdog.update(211ms, false, false), false);
}

TEST(CommandWatchdog, ReenableRequiresANewCommand) {
  amr_simulation::CommandWatchdog watchdog(200ms);
  watchdog.update(0ms, true, false);
  EXPECT_EQ(watchdog.update(201ms, false, false), false);
  EXPECT_FALSE(watchdog.update(250ms, false, false).has_value());
  EXPECT_EQ(watchdog.update(251ms, true, false), true);
}

TEST(CommandWatchdog, TimeRewindFailsClosed) {
  amr_simulation::CommandWatchdog watchdog(200ms);
  watchdog.update(100ms, true, false);
  EXPECT_EQ(watchdog.update(50ms, false, true), false);
  EXPECT_FALSE(watchdog.update(60ms, false, false).has_value());
}
