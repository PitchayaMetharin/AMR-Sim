# State values. Zero is intentionally UNKNOWN.
uint8 UNKNOWN=0
uint8 UNAVAILABLE=1
uint8 READY=2
uint8 FAULT=3

# Reason values. Zero is intentionally UNAVAILABLE.
uint16 REASON_UNAVAILABLE=0
uint16 REASON_READY=1
uint16 REASON_ODOMETRY_MISSING_OR_STALE=2
uint16 REASON_JOINT_STATES_MISSING_OR_STALE=3
uint16 REASON_FAULT=4

std_msgs/Header header
uint32 sequence
bool valid
uint32 source_boot_id
uint8 state
uint16 reason
