# EKF2 GNSS rejoin (EKF2_GPS_MODE = Strict)
#
# Published for every GNSS sample at the fusion time horizon. Holds the values that decide
# whether GNSS may (re)join, so that logs stay meaningful if the thresholds change.

uint64 timestamp                    # time since system start (microseconds)
uint64 timestamp_sample             # time of the GNSS sample (microseconds)

uint8 pilot_state                   # state shown to the pilot
uint8 PILOT_STATE_GPS_BAD = 0       # no data, failing checks, health time running or rejoin not possible
uint8 PILOT_STATE_WAITING = 1       # checks pass, waiting for the filters to agree
uint8 PILOT_STATE_IN_USE  = 2       # GNSS is being fused

bool in_use                         # any of GNSS velocity, position or height is fused
bool shadow                         # GNSS fusion disabled (EKF2_GPS_CTRL = 0): evaluated and logged only, never fused
bool shadow_would_join              # shadow: all filters ready on this sample, GNSS would have joined
uint16 shadow_join_count            # shadow: number of times shadow_would_join became true
bool preconditions_ok               # tilt aligned, non-GNSS horizontal aiding, vertical position valid
bool checks_passed                  # strict checks passing including the health time
bool sample_failed_checks           # the last sample failed the strict checks
float32 time_since_check_fail       # [s] time since the last failed sample

# window and filters (while not in use)
uint8 window_samples
float32 window_span                 # [s]
bool heading_required
bool pos_required
bool hgt_required
bool heading_ready
bool pos_ready
bool hgt_ready

uint8 heading_source                # source of the heading used for the rejoin
uint8 HEADING_SOURCE_NONE = 0
uint8 HEADING_SOURCE_FIT  = 1       # fitted from GNSS vs EKF velocity
uint8 HEADING_SOURCE_EKF  = 2       # EKF heading still good

float32 heading_fit                 # [rad] heading correction fitted from GNSS velocity
float32 heading_fit_std             # [rad] 1-sigma of the fitted correction
float32 ekf_heading_std             # [rad] 1-sigma of the EKF heading
float32 speed_rms                   # [m/s] RMS EKF horizontal speed over the window
float32 vel_resid_fit               # [m/s] RMS velocity residual per axis with the fitted correction
float32 vel_resid_zero              # [m/s] RMS velocity residual per axis without correction
float32 vel_acc_rms                 # [m/s] RMS GNSS velocity accuracy used for fusion

float32[2] pos_offset               # [m] carried-forward GNSS position - EKF position, NE (NaN without origin)
float32 pos_scatter                 # [m] RMS scatter per axis of the carried-forward GNSS positions
float32 pos_acc_rms                 # [m] RMS GNSS horizontal accuracy used for fusion

float32 hgt_offset                  # [m] carried-forward GNSS altitude - EKF altitude
float32 hgt_scatter                 # [m] RMS scatter of the carried-forward GNSS altitudes
float32 hgt_acc_rms                 # [m] RMS GNSS vertical accuracy used for fusion

# while in use
float32 time_since_vel_fuse         # [s]
float32 time_since_pos_fuse         # [s]
float32 time_since_hgt_fuse         # [s]

# last join
float32 join_heading_correction     # [rad]
uint8 join_heading_source           # HEADING_SOURCE_* used at the last join
float32[2] join_pos_offset          # [m]
float32 join_hgt_offset             # [m]

uint16 join_count
uint16 stop_count
uint16 heading_reset_count
uint16 pos_reset_count
uint16 hgt_reset_count
uint16 window_clear_count           # window restarted for a reason other than a join

uint8 last_stop_reason
uint8 STOP_REASON_NONE          = 0
uint8 STOP_REASON_CHECKS        = 1 # sample failed the strict checks
uint8 STOP_REASON_NO_DATA       = 2 # no GNSS data for EKF2_GPS_RJ_TOUT
uint8 STOP_REASON_REJECTED      = 3 # not fused for EKF2_GPS_RJ_TOUT
uint8 STOP_REASON_PREREQUISITES = 4 # tilt or yaw alignment lost
uint8 STOP_REASON_NOT_INTENDED  = 5 # GNSS fusion disabled

# TOPICS estimator_gnss_rejoin
