std_msgs/Header header

# IR Camera Flag State
uint8 IRFLAG_OPEN = 0
uint8 IRFLAG_CLOSE = 1
uint8 IRFLAG_OPENING = 2
uint8 IRFLAG_CLOSING = 3
uint8 IRFLAG_ERROR = 4
uint8 IRFLAG_INITIALIZING = 5

# IR Camera Temp Range
uint8 IRRANGE_LOW = 0
uint8 IRRANGE_MED = 1
uint8 IRRANGE_HIGH = 2

uint8 flag_state
uint8 temp_range