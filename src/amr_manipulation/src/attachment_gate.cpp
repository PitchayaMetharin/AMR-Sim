#include "amr_manipulation/attachment_gate.hpp"

#include <cmath>

namespace amr_manipulation
{

bool AttachmentGate::configured_product(const int product_id)
{
  return product_id >= 101 && product_id <= 103;
}

AttachmentDecision AttachmentGate::evaluate_attach(
  const AttachmentEvidence & evidence) const
{
  if (!configured_product(evidence.product_id)) {
    return AttachmentDecision::UNKNOWN_PRODUCT;
  }
  if (attached_product_id_) {
    return *attached_product_id_ == evidence.product_id ?
           AttachmentDecision::ALREADY_ATTACHED : AttachmentDecision::WRONG_PRODUCT;
  }
  if (!std::isfinite(evidence.position_error_m) ||
    !std::isfinite(evidence.orientation_error_rad) ||
    evidence.position_error_m < 0.0 || evidence.orientation_error_rad < 0.0 ||
    evidence.position_error_m > kMaximumPositionErrorM ||
    evidence.orientation_error_rad > kMaximumOrientationErrorRad)
  {
    return AttachmentDecision::POSE_OUT_OF_TOLERANCE;
  }
  if (!evidence.left_contact || !std::isfinite(evidence.left_contact_age_s) ||
    evidence.left_contact_age_s < 0.0 ||
    evidence.left_contact_age_s > kMaximumContactAgeS)
  {
    return AttachmentDecision::LEFT_CONTACT_MISSING_OR_STALE;
  }
  if (!evidence.right_contact || !std::isfinite(evidence.right_contact_age_s) ||
    evidence.right_contact_age_s < 0.0 ||
    evidence.right_contact_age_s > kMaximumContactAgeS)
  {
    return AttachmentDecision::RIGHT_CONTACT_MISSING_OR_STALE;
  }
  return AttachmentDecision::ACCEPT;
}

AttachmentDecision AttachmentGate::evaluate_detach(
  const int product_id, const double nearest_dispatch_position_error_m) const
{
  if (!configured_product(product_id)) {
    return AttachmentDecision::UNKNOWN_PRODUCT;
  }
  if (!attached_product_id_) {
    return AttachmentDecision::NOT_ATTACHED;
  }
  if (*attached_product_id_ != product_id) {
    return AttachmentDecision::WRONG_PRODUCT;
  }
  if (!std::isfinite(nearest_dispatch_position_error_m) ||
    nearest_dispatch_position_error_m < 0.0 ||
    nearest_dispatch_position_error_m > kMaximumDispatchPositionErrorM)
  {
    return AttachmentDecision::DISPATCH_POSE_OUT_OF_TOLERANCE;
  }
  return AttachmentDecision::ACCEPT;
}

bool AttachmentGate::confirm_attached(const int product_id)
{
  if (!configured_product(product_id) || attached_product_id_) {
    return false;
  }
  attached_product_id_ = product_id;
  return true;
}

bool AttachmentGate::confirm_detached(const int product_id)
{
  if (!attached_product_id_ || *attached_product_id_ != product_id) {
    return false;
  }
  attached_product_id_.reset();
  return true;
}

std::optional<int> AttachmentGate::attached_product_id() const
{
  return attached_product_id_;
}

}  // namespace amr_manipulation
