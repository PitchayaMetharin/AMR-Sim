#include <gtest/gtest.h>

#include <limits>

#include "amr_manipulation/attachment_gate.hpp"

using amr_manipulation::AttachmentDecision;
using amr_manipulation::AttachmentEvidence;
using amr_manipulation::AttachmentGate;

namespace
{
AttachmentEvidence valid_evidence(const int product_id)
{
  return {product_id, 0.029, 0.149, true, true, 0.099, 0.099};
}
}  // namespace

TEST(AttachmentGate, AcceptsOnlyConfiguredProductsWithPoseAndBilateralContact)
{
  for (const int product_id : {101, 102, 103}) {
    AttachmentGate gate;
    EXPECT_EQ(gate.evaluate_attach(valid_evidence(product_id)), AttachmentDecision::ACCEPT);
  }

  AttachmentGate gate;
  auto evidence = valid_evidence(999);
  EXPECT_EQ(gate.evaluate_attach(evidence), AttachmentDecision::UNKNOWN_PRODUCT);
  evidence = valid_evidence(101);
  evidence.position_error_m = 0.031;
  EXPECT_EQ(gate.evaluate_attach(evidence), AttachmentDecision::POSE_OUT_OF_TOLERANCE);
  evidence = valid_evidence(101);
  evidence.orientation_error_rad = 0.151;
  EXPECT_EQ(gate.evaluate_attach(evidence), AttachmentDecision::POSE_OUT_OF_TOLERANCE);
  evidence = valid_evidence(101);
  evidence.left_contact = false;
  EXPECT_EQ(
    gate.evaluate_attach(evidence),
    AttachmentDecision::LEFT_CONTACT_MISSING_OR_STALE);
  evidence = valid_evidence(101);
  evidence.right_contact_age_s = 0.101;
  EXPECT_EQ(
    gate.evaluate_attach(evidence),
    AttachmentDecision::RIGHT_CONTACT_MISSING_OR_STALE);
  evidence = valid_evidence(101);
  evidence.position_error_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(gate.evaluate_attach(evidence), AttachmentDecision::POSE_OUT_OF_TOLERANCE);
}

TEST(AttachmentGate, StateChangesOnlyAfterMatchingGazeboConfirmation)
{
  AttachmentGate gate;
  EXPECT_FALSE(gate.attached_product_id());
  EXPECT_FALSE(gate.confirm_attached(999));
  EXPECT_FALSE(gate.attached_product_id());
  EXPECT_TRUE(gate.confirm_attached(102));
  ASSERT_TRUE(gate.attached_product_id());
  EXPECT_EQ(*gate.attached_product_id(), 102);
  EXPECT_FALSE(gate.confirm_attached(102));
  EXPECT_EQ(
    gate.evaluate_attach(valid_evidence(101)), AttachmentDecision::WRONG_PRODUCT);
  EXPECT_FALSE(gate.confirm_detached(101));
  EXPECT_TRUE(gate.attached_product_id());
  EXPECT_TRUE(gate.confirm_detached(102));
  EXPECT_FALSE(gate.attached_product_id());
}

TEST(AttachmentGate, DetachmentRequiresMatchingProductAtDispatchPose)
{
  AttachmentGate gate;
  EXPECT_EQ(gate.evaluate_detach(101, 0.0), AttachmentDecision::NOT_ATTACHED);
  ASSERT_TRUE(gate.confirm_attached(101));
  EXPECT_EQ(gate.evaluate_detach(102, 0.0), AttachmentDecision::WRONG_PRODUCT);
  EXPECT_EQ(
    gate.evaluate_detach(101, 0.031),
    AttachmentDecision::DISPATCH_POSE_OUT_OF_TOLERANCE);
  EXPECT_EQ(gate.evaluate_detach(101, 0.030), AttachmentDecision::ACCEPT);
}
