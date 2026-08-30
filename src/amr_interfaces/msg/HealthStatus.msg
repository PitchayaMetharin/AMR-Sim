# Aggregate simulation health. Zero is intentionally UNKNOWN.
uint8 UNKNOWN=0
uint8 DEGRADED=1
uint8 HEALTHY=2
uint8 FAULT=3

# Reason values. Zero is intentionally UNAVAILABLE.
uint16 REASON_UNAVAILABLE=0
uint16 REASON_HEALTHY=1
uint16 REASON_EVIDENCE_MISSING_OR_STALE=2
uint16 REASON_INVALID_EVIDENCE=3
uint16 REASON_BACKWARD_TIME=4
uint16 REASON_SOURCE_NOT_READY=5
uint16 REASON_SOURCE_FAULT=6

std_msgs/Header header
uint32 sequence
bool valid
uint32 source_boot_id
uint8 state
uint16 reason
bool base_ready
