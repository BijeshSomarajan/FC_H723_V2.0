#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POIHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POIHELPER_H_

#include <stdint.h>

#define POSITION_POI_DEFAULT_RADIUS      3.0f    // m
#define POSITION_POI_DEFAULT_CCW         1
#define POSITION_POI_DEFAULT_ITERATIONS  1

#define POSITION_POI_MIN_RADIUS          3.0f    // m
#define POSITION_POI_MAX_RADIUS          50.0f   // m
#define POSITION_POI_MAX_ITERATIONS      10
#define POSITION_POI_MAX_ARC             500.0f  // m, total orbit path
#define POSITION_POI_MAX_ENTRY_DIST      30.0f   // m, max distance from the drone to the circle
#define POSITION_POI_MAX_HOME_DIST       120.0f  // m, centre distance from home + radius (radio range guard)
#define POSITION_POI_START_MAX_SPEED     0.5f    // m/s, must be at rest to start
#define POSITION_POI_RETURN_TO_START     1       // 1 = fly back to where it started, 0 = stay at the end point

#define POSITION_POI_SPEED               2.0f    // m/s, leg speed and orbit speed ceiling
#define POSITION_POI_BRAKE_DECEL         0.75f   // m/s²
#define POSITION_POI_MAX_ACCEL           5.0f    // m/s²
#define POSITION_POI_FF_ACC_MAX          1.0f    // m/s², acceleration feedforward cap
#define POSITION_POI_LAT_ACC             0.8f    // m/s², orbit speed <= sqrt(a*R). Keep <= POSITION_POI_FF_ACC_MAX
#define POSITION_POI_YAW_FRACTION        0.8f    // orbit rate <= this fraction of the heading slew rate
#define POSITION_POI_RADIAL_KP           0.5f    // 1/s
#define POSITION_POI_RADIAL_VMAX         1.0f    // m/s
#define POSITION_POI_MIN_DIST            0.5f    // m, direction to the centre is undefined closer than this
#define POSITION_POI_ARC_STOP            0.15f   // m, start and end speed profiles reach zero this far from the ends
#define POSITION_POI_DONE_ARC            0.5f    // m
#define POSITION_POI_DONE_SPEED          0.3f    // m/s

/* Entry and return legs */
#define POSITION_POI_STOP_RADIUS         0.15f   // m, leg speed profile reaches zero here
#define POSITION_POI_REACH_RADIUS        0.5f    // m
#define POSITION_POI_REACH_SPEED         0.2f    // m/s
#define POSITION_POI_DWELL_TIME          0.6f    // s
#define POSITION_POI_DWELL_SPEED         0.3f    // m/s, fallback ceiling (wind)
#define POSITION_POI_LEG_TIMEOUT         60.0f   // s

/* Heading */
#define POSITION_POI_ALIGN_TOL_DEG       5.0f    // nose error that counts as aligned
#define POSITION_POI_ALIGN_REF_DEG       1.0f    // heading reference must have arrived within this
#define POSITION_POI_ALIGN_STABLE_TIME   0.3f    // s
#define POSITION_POI_ALIGN_TIMEOUT       25.0f   // s, above the heading helper's own worst case
#define POSITION_POI_HDG_FULL_DEG        10.0f   // orbit at full speed below this nose error
#define POSITION_POI_HDG_ZERO_DEG        30.0f   // orbit speed falls to zero at this nose error

/* Settle before return */
#define POSITION_POI_SETTLE_SPEED        0.2f    // m/s
#define POSITION_POI_SETTLE_TIME         0.3f    // s
#define POSITION_POI_SETTLE_TIMEOUT      5.0f    // s

uint8_t decodePOIValue(float value, float *radius, uint8_t *iterations, uint8_t *ccw);   // packed value from the mission operation
uint8_t startPOIOperation(double anchorLat, double anchorLon, float radius, uint8_t ccw, uint8_t iterations);   // mission start, 1 = accepted
uint8_t isPOIOperationComplete(float dt);   // runs one step; returns 1 when back at the anchor (or nothing is running)
void cancelPOIOperation(void);              // mission reset: controlled stop
void poiOperationRelock(void);              // mission re-locked after a stick override: resume

void handlePOI(float dt);                   // radio switch path (position manager)
void resetPOIStates(void);
void setPOIParameters(float radius, uint8_t ccw, uint8_t iterations);
uint8_t getPOIPhase(void);

/* Packed operation value: bits 0-9 radius in 0.1 m, bits 10-14 revolutions (0 reads as 1), bit 15 direction (1 = ccw), bits 16-23 reserved */
#define POI_VALUE_RADIUS_MASK            0x000003FFu
#define POI_VALUE_REV_SHIFT              10
#define POI_VALUE_REV_MASK               0x1Fu
#define POI_VALUE_DIR_BIT                0x00008000u
#define POI_VALUE_RESERVED_MASK          0x00FF0000u
#define POSITION_POI_ANCHOR_ENTRY_MARGIN 5.0f    // m, added to the radius for the entry leg of an anchored POI

#endif
