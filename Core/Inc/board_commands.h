#pragma once
/* Local control command IDs are independent of the network implementation. */
enum valve_commands {
    CMD_PILOT_OPEN = 0,
    CMD_NORMALLY_OPEN_OPEN = 1,
    CMD_DUMP_OPEN = 2,
    CMD_PILOT_CLOSE = 3,
    CMD_NORMALLY_OPEN_CLOSE = 4,
    CMD_DUMP_CLOSE = 5,
    CMD_SEQUENCE = 6,
    CMD_ABORT = 7,
};
enum actuator_commands {
    CMD_IGNITER_SEQUENCE = 14,
};
