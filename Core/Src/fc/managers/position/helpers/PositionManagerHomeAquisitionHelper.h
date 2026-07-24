#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHOMEAQUISITIONHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHOMEAQUISITIONHELPER_H_

/* =============================================================================
 *  HOME POSITION ACQUISITION - convergence gated
 * =============================================================================
 *
 *  PROBLEM WITH A FIXED-N AVERAGE
 *  ------------------------------
 *  Averaging the first N fixes assumes the fixes are already stationary and
 *  only noisy. They are not: after a fix is declared valid the solution keeps
 *  SLEWING for tens of seconds as the receiver pulls in more satellites,
 *  resolves multipath and settles its filters. Averaging across a slew just
 *  produces the mean of the slew - biased toward wherever the receiver started,
 *  by roughly half the total drift. Taking MORE samples makes this worse, not
 *  better: it averages in more of the early, wrong region.
 *
 *  STRATEGY
 *  --------
 *  Buffer one sample per second over a sliding window and lock home only when
 *  the window shows the solution has STOPPED MOVING. Three independent gates,
 *  all of which must pass:
 *
 *    1. RECEIVER CONFIDENCE  hAcc/vAcc below threshold. Cheap, and it is the
 *       receiver's own opinion - do not average fixes it already distrusts.
 *
 *    2. SPREAD               RMS deviation from the window mean. Catches a
 *       noisy-but-centred solution: jumping around, no useful mean yet.
 *
 *    3. DRIFT                mean of the OLDER half vs mean of the NEWER half.
 *       This is the one that detects slew. A converging solution has small
 *       spread within each half but a real offset BETWEEN them; only a settled
 *       solution has both halves agreeing.
 *
 *  Gate 3 is what a fixed-N average cannot do. Spread alone passes happily on a
 *  smooth, steady drift - every sample sits close to the running mean, so the
 *  scatter looks fine while the whole cloud is still moving.
 *
 *  On a spread/drift failure the window SLIDES (oldest dropped) and we retry
 *  next second. On a confidence failure the window is CLEARED - samples taken
 *  under a bad fix must not be averaged with good ones.
 *
 *  Home altitude uses the same window; hMSL settles slower than lat/lon on most
 *  receivers, so it gets its own, looser gates.
 *
 *  WHY RMS AND NOT MAX DEVIATION
 *  -----------------------------
 *  The obvious spread metric - max |sample - mean| - is a trap here, because it
 *  GROWS WITH WINDOW LENGTH. Simulated at sigma = 0.35 m:
 *
 *      N=10  median max-dev 0.86 m   RMS-dev 0.51 m
 *      N=20  median max-dev 0.93 m   RMS-dev 0.50 m
 *      N=40  median max-dev 1.01 m   RMS-dev 0.50 m
 *
 *  So lengthening the window to detect a SLOWER slew would silently tighten a
 *  max-based gate, and the tuning advice below would fight itself. RMS is
 *  window-length independent (~sigma*sqrt(2) for 2-D noise) and far less
 *  sensitive to a single outlying fix.
 *
 *  KEEPING THE GATES MUTUALLY CONSISTENT
 *  -------------------------------------
 *  MAX_HACC and MAX_SPREAD_RMS must describe the SAME receiver, or home never
 *  locks on the settled path and every acquisition falls through to the
 *  timeout. Simulated P(lock) per attempt at zero slew, MAX_SPREAD_RMS 1.00:
 *
 *      sigma 0.25 m -> 99%      sigma 0.50 m -> 70%
 *      sigma 0.35 m -> 93%      sigma 0.75 m -> 20%
 *
 *  and the drift gate still does its job - at 0.05 m/s residual slew, P(lock)
 *  falls to 8-17% across that whole range.
 *
 *  A MAX_HACC of 2.0 m admits receivers with sigma up to ~1 m. If yours is that
 *  noisy, either tighten MAX_HACC or widen MAX_SPREAD_RMS - do not leave them
 *  contradicting each other.
 *
 *  HOW TO SET THE THRESHOLDS (the only honest way)
 *  -----------------------------------------------
 *  Sit on the ground with a good fix for two minutes. Log lat/lon, convert to
 *  metres, discard the first 30 s, and take the standard deviation of what is
 *  left - that is your sigma. Then:
 *
 *      MAX_SPREAD_RMS ~ 2.0 * sigma        (passes ~90% of settled windows)
 *      MAX_DRIFT      ~ 0.7 * sigma        (rejects slew, tolerates noise)
 *      MAX_HACC        set so it only admits fixes the spread gate can pass
 *
 *  Re-measure after any antenna, ground-plane or mounting change.
 *
 *  NOTE ON THE TIMEOUT
 *  -------------------
 *  The timeout only applies once the window is FULL. If the confidence gate
 *  keeps clearing the window it will never fill, and home will never be set at
 *  all - deliberately, because a site that cannot hold WINDOW_LEN consecutive
 *  good fixes has no business defining an RTH target. The timeout covers the
 *  other case: a full window that is stable enough to keep but never quite
 *  settles inside the gates.
 * ========================================================================== */

#include <sys/_stdint.h>

/* Window length x sample period sets how long a slew you can detect. 20 x 1 s
 * covers the usual 15-25 s settling of a cold-ish fix. Lengthen if logs show
 * home still moving after lock - safe to do now that the spread gate is RMS
 * based and no longer tightens with N. */
#define POSITION_MGR_HOME_WINDOW_LEN        20
#define POSITION_MGR_HOME_SAMPLE_PERIOD     1.0f   // s between buffered samples

/* Receiver confidence gate. Do not set below what the module achieves on a
 * good day or home will never lock. Must be consistent with MAX_SPREAD_RMS -
 * see "KEEPING THE GATES MUTUALLY CONSISTENT" above. */
#define POSITION_MGR_HOME_MAX_HACC          2.0f   // m
#define POSITION_MGR_HOME_MAX_VACC          3.0f   // m

/* Spread gate: RMS |sample - window mean|. Window-length independent.
 * Rule of thumb: ~2.0 * receiver sigma. */
#define POSITION_MGR_HOME_MAX_SPREAD_RMS    1.00f  // m
#define POSITION_MGR_HOME_MAX_SPREAD_RMS_Z  2.00f  // m  (hMSL is noisier)

/* Drift gate: |mean(newer half) - mean(older half)|. THE slew detector.
 * Tighten to demand a more settled fix, at the cost of a longer wait.
 * Rule of thumb: ~0.7 * receiver sigma. */
#define POSITION_MGR_HOME_MAX_DRIFT         0.35f  // m
#define POSITION_MGR_HOME_MAX_DRIFT_Z       0.60f  // m

/* Hard cap: once the window is full, lock on the newer half after this long
 * even if the gates never pass, so a marginal site cannot block arming
 * forever. Set to 0 to disable and require a clean lock. See "NOTE ON THE
 * TIMEOUT" - this does NOT cover a window that never fills. */
#define POSITION_MGR_HOME_TIMEOUT           90.0f  // s

typedef struct {
	double lat;
	double lon;
	float hMSL;
} HOME_SAMPLE;

void resetHomePositionAcquisition(void);
uint8_t updateHomePositionAcquisition(float dt);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMANAGERHOMEAQUISITIONHELPER_H_ */
