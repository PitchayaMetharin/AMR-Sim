std_msgs/Header header

uint8 STARTING=0
uint8 STOWED_EMPTY=1
uint8 STOWED_LOADED=2
uint8 MOVING=3
uint8 DEPLOYED=4
uint8 FAULT=5

uint32 source_boot_id
uint32 sequence
bool valid
uint8 state
bool base_motion_allowed
bool product_attached
string product_id
string detail
