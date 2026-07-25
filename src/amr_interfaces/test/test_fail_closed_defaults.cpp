#include <gtest/gtest.h>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/gate_status.hpp"
#include "amr_interfaces/msg/plc_connection_status.hpp"
#include "amr_interfaces/msg/plc_state.hpp"
#include "amr_interfaces/srv/request_motion_enable.hpp"
#include "amr_interfaces/srv/request_reset.hpp"

TEST(FailClosedDefaults, StatusMessagesAreInvalidAndUnknown)
{
  const amr_interfaces::msg::BaseStatus base;
  EXPECT_EQ(base.sequence, 0U);
  EXPECT_FALSE(base.valid);
  EXPECT_EQ(base.source_boot_id, 0U);
  EXPECT_EQ(base.state, amr_interfaces::msg::BaseStatus::UNKNOWN);
  EXPECT_EQ(base.reason, amr_interfaces::msg::BaseStatus::REASON_UNAVAILABLE);

  const amr_interfaces::msg::PlcConnectionStatus connection;
  EXPECT_EQ(connection.sequence, 0U);
  EXPECT_FALSE(connection.valid);
  EXPECT_EQ(connection.source_boot_id, 0U);
  EXPECT_EQ(connection.state, amr_interfaces::msg::PlcConnectionStatus::UNKNOWN);
  EXPECT_EQ(
    connection.reason,
    amr_interfaces::msg::PlcConnectionStatus::REASON_UNAVAILABLE);
}

TEST(FailClosedDefaults, PermissionMessagesCannotDefaultToPermission)
{
  const amr_interfaces::msg::GateStatus gate;
  EXPECT_FALSE(gate.valid);
  EXPECT_EQ(gate.state, amr_interfaces::msg::GateStatus::UNKNOWN);
  EXPECT_EQ(gate.reason, amr_interfaces::msg::GateStatus::REASON_UNAVAILABLE);
  EXPECT_FALSE(gate.raw_permission);

  const amr_interfaces::msg::PlcState plc;
  EXPECT_FALSE(plc.valid);
  EXPECT_EQ(plc.state, amr_interfaces::msg::PlcState::UNKNOWN);
  EXPECT_EQ(plc.reason, amr_interfaces::msg::PlcState::REASON_UNAVAILABLE);
  EXPECT_FALSE(plc.raw_permission);
}

TEST(FailClosedDefaults, GatewayRequestsAndResponsesAreInactive)
{
  const amr_interfaces::srv::RequestMotionEnable::Request enable_request;
  EXPECT_EQ(enable_request.sequence, 0U);
  EXPECT_EQ(enable_request.source_boot_id, 0U);
  EXPECT_FALSE(enable_request.valid);
  EXPECT_FALSE(enable_request.motion_enable_request);

  const amr_interfaces::srv::RequestMotionEnable::Response enable_response;
  EXPECT_FALSE(enable_response.accepted_for_delivery);
  EXPECT_EQ(enable_response.acknowledged_sequence, 0U);
  EXPECT_EQ(
    enable_response.reason,
    amr_interfaces::srv::RequestMotionEnable::Response::REASON_UNAVAILABLE);

  const amr_interfaces::srv::RequestReset::Request reset_request;
  EXPECT_EQ(reset_request.sequence, 0U);
  EXPECT_EQ(reset_request.source_boot_id, 0U);
  EXPECT_FALSE(reset_request.valid);
  EXPECT_FALSE(reset_request.reset_request);

  const amr_interfaces::srv::RequestReset::Response reset_response;
  EXPECT_FALSE(reset_response.accepted_for_delivery);
  EXPECT_EQ(reset_response.acknowledged_sequence, 0U);
  EXPECT_EQ(
    reset_response.reason,
    amr_interfaces::srv::RequestReset::Response::REASON_UNAVAILABLE);
}
