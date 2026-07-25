# State values. Zero is intentionally UNKNOWN.
uint8 UNKNOWN=0
uint8 DISCONNECTED=1
uint8 READY_INHIBITED=2
uint8 FAULT=3

# Reason values. Zero is intentionally UNAVAILABLE.
uint16 REASON_UNAVAILABLE=0

std_msgs/Header header
uint32 sequence
bool valid
uint32 source_boot_id
uint8 state
uint16 reason
