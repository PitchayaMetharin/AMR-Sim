#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace amr_manipulation
{

enum class AttachmentDecision : std::uint8_t
{
  ACCEPT = 0,
  UNKNOWN_PRODUCT,
  ALREADY_ATTACHED,
  WRONG_PRODUCT,
  POSE_OUT_OF_TOLERANCE,
  LEFT_CONTACT_MISSING_OR_STALE,
  RIGHT_CONTACT_MISSING_OR_STALE,
  NOT_ATTACHED,
  DISPATCH_POSE_OUT_OF_TOLERANCE,
};

struct AttachmentEvidence
{
  int product_id{0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  bool left_contact{false};
  bool right_contact{false};
  double left_contact_age_s{0.0};
  double right_contact_age_s{0.0};
};

class AttachmentGate
{
public:
  static constexpr double kMaximumPositionErrorM = 0.030;
  static constexpr double kMaximumOrientationErrorRad = 0.15;
  static constexpr double kMaximumContactAgeS = 0.100;
  static constexpr double kMaximumDispatchPositionErrorM = 0.030;

  AttachmentDecision evaluate_attach(const AttachmentEvidence & evidence) const;
  AttachmentDecision evaluate_detach(
    int product_id, double nearest_dispatch_position_error_m) const;

  bool confirm_attached(int product_id);
  bool confirm_detached(int product_id);
  std::optional<int> attached_product_id() const;

private:
  static bool configured_product(int product_id);
  std::optional<int> attached_product_id_;
};

}  // namespace amr_manipulation
