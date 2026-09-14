std_msgs/Header header

uint8 MANUAL=0
uint8 AUTONOMOUS=1

uint32 sequence
uint8 mode
uint8 phase
bool active
uint32 queue_depth
string pickup_station_id
string destination_station_id
string product_id
bool product_attached
uint8 last_outcome
string detail

# Sequence state is appended for wire compatibility with the existing fields.
bool sequence_active
uint32 current_cycle
uint32 sequence_index
uint32 completed_jobs
uint32 completed_cycles
string final_station_id
bool graceful_stop_requested
bool fault_latched
