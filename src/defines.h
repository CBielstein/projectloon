#ifndef DEFINES_H
#define DEFINES_H

// A central file to keep all of our constants to enable easy modification

// LTE has 20km radius on ground, 20km in the air, which yields 28.284km signal range from the balloon
// Special thanks to Hudson Bielstein for the computation
#define LTE_SIGNAL_RADIUS 28284.0

// ISM between balloons has 40km radius in air
#define ISM_SIGNAL_RADIUS 40000.0

// Balloon top speed is ~150 mph ~= 67 m/s
#define BALLOON_TOP_SPEED 67.0

// Length in seconds between updates of balloon position
#define BALLOON_POSITION_UPDATE_RATE 0.5

// heartbeat interval
#define BALLOON_HEARTBEAT_INTERVAL 1.0

// a positive multiple of BALLON_HEARTBEAT_INTERVAL to use for the ETX interval
#define BALLOON_ETX_MULTIPLE 10.0

// the percentage of jitter applied by the Jitter() function
#define JITTER_PERCENT 25.0

#endif
