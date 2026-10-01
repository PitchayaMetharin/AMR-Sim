// Copyright 2018, Open Source Robotics Foundation, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_interface.hpp"

class MockCreateTimer final : public tf2_ros::CreateTimerInterface
{
public:
  MockCreateTimer()
  : timer_handle_index_(0)
  {
  }

  tf2_ros::TimerHandle
  createTimer(
    rclcpp::Clock::SharedPtr clock,
    const tf2::Duration & period,
    tf2_ros::TimerCallbackType callback) override
  {
    (void) clock;
    (void) period;
    const auto timer_handle = timer_handle_index_++;
    timer_to_callback_map_[timer_handle] = callback;
    return timer_handle;
  }

  void
  cancel(const tf2_ros::TimerHandle & timer_handle) override
  {
    (void) timer_handle;
  }

  void
  reset(const tf2_ros::TimerHandle & timer_handle) override
  {
    (void) timer_handle;
  }

  void
  remove(const tf2_ros::TimerHandle & timer_handle) override
  {
    // Don't actually remove the timer to avoid a race with the test callback.
    (void) timer_handle;
  }

private:
  tf2_ros::TimerHandle timer_handle_index_;
  std::unordered_map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> timer_to_callback_map_;
};

// Reproduces the ABBA deadlock:
//
//   Thread A - waitForTransform:
//     holds timer_to_request_map_mutex_
//       -> BufferCore::addTransformableRequest
//         -> waits for transformable_requests_mutex_
//
//   Thread B - setTransform -> testTransformableRequests:
//     holds transformable_requests_mutex_
//       -> waitForTransform ready-callback
//         -> waits for timer_to_request_map_mutex_
TEST(test_tf2_ros_deadlock, wait_for_transform_does_not_deadlock_with_set_transform)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  auto mock_create_timer = std::make_shared<MockCreateTimer>();
  buffer.setCreateTimerInterface(mock_create_timer);

  const tf2::TimePoint time_point = tf2::timeFromSec(1.0);
  const std::string target_frame = "foo";
  const std::string source_frame = "bar";

  std::promise<void> in_transformable_callback;
  std::promise<void> waiter_finished;
  auto waiter_finished_future = waiter_finished.get_future();
  std::thread waiter_thread;

  // This callback starts a concurrent waitForTransform while
  // testTransformableRequests is processing the first request. The concurrent
  // request must be able to wait for the request mutex without deadlocking the
  // callback that is already being dispatched.
  auto gate_cb =
    [&buffer, &in_transformable_callback, &waiter_thread, &waiter_finished, time_point,
      target_frame](
    tf2::TransformableRequestHandle, const std::string &, const std::string &,
    tf2::TimePoint, tf2::TransformableResult)
    {
      waiter_thread = std::thread(
        [&buffer, &in_transformable_callback, &waiter_finished, time_point, target_frame]()
        {
          in_transformable_callback.get_future().wait();
          buffer.waitForTransform(
            target_frame, "other", time_point, tf2::durationFromSec(1.0),
            [](const tf2_ros::TransformStampedFuture &) {});
          waiter_finished.set_value();
        });
      in_transformable_callback.set_value();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

  ASSERT_NE(
    buffer.addTransformableRequest(gate_cb, target_frame, source_frame, time_point),
    0u);

  bool wait_callback_called = false;
  auto future = buffer.waitForTransform(
    target_frame, source_frame, time_point, tf2::durationFromSec(1.0),
    [&wait_callback_called](const tf2_ros::TransformStampedFuture &) {
      wait_callback_called = true;
    });

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = target_frame;
  transform.header.stamp.sec = 1;
  transform.child_frame_id = source_frame;
  transform.transform.rotation.w = 1.0;

  std::promise<void> set_transform_done;
  std::thread setter([&buffer, &transform, &set_transform_done]() {
      EXPECT_TRUE(buffer.setTransform(transform, "unittest"));
      set_transform_done.set_value();
    });

  const auto set_status = set_transform_done.get_future().wait_for(std::chrono::seconds(5));
  EXPECT_EQ(set_status, std::future_status::ready) <<
    "Deadlock between waitForTransform (timer_to_request_map_mutex_ -> "
    "transformable_requests_mutex_) and testTransformableRequests "
    "(transformable_requests_mutex_ -> timer_to_request_map_mutex_).";
  if (set_status != std::future_status::ready) {
    std::_Exit(1);
  }

  setter.join();
  ASSERT_TRUE(waiter_thread.joinable());
  const auto waiter_status = waiter_finished_future.wait_for(std::chrono::seconds(1));
  EXPECT_EQ(waiter_status, std::future_status::ready);
  if (waiter_status != std::future_status::ready) {
    std::_Exit(1);
  }
  waiter_thread.join();

  EXPECT_TRUE(wait_callback_called);
  EXPECT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
}
