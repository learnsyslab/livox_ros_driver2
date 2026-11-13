# Time synchronization (PTP / GPS) status of one Livox lidar.
# Published periodically on "livox/time_sync_status", one message per connected lidar.
#
# Two independent sources are combined:
#  (a) the lidar's own status, queried over the control channel (keys 0x8009..0x800C), and
#  (b) the timestamp type stamped into every point cloud / IMU packet the lidar streams.

uint8 SYNC_NONE = 0   # free running: timestamps are time since power-on
uint8 SYNC_PTP  = 1   # PTP (IEEE 1588v2) / gPTP
uint8 SYNC_GPS  = 2   # GPS (PPS + GPRMC)

std_msgs/Header header       # stamp: host time this status was assembled
string lidar_ip

# ---------------- Verdict ----------------
# True if the lidar reports PTP/GPS sync, its last_sync_time keeps advancing (i.e. it is
# still receiving sync messages), and the streamed packets carry synchronized timestamps.
bool synchronized
# True if last_sync_time changed within the last `time_sync_timeout` seconds (host clock),
# i.e. the lidar is actively receiving Sync/Follow_Up messages from the PTP master.
bool receiving_sync_messages
string summary               # human readable one-liner

# ---------------- (a) Lidar-reported status ----------------
bool status_valid            # at least one status response has been received
float64 status_age           # [s] host time since the last status response
uint8 time_sync_type         # 0x800C, one of SYNC_*
uint64 local_time_now        # 0x8009, lidar local clock [ns]
uint64 last_sync_time        # 0x800A, master time t1 of the last PTP Follow_Up (GPS: PPS edge) [ns]
int64 time_offset            # 0x800B, lidar local time minus sync source time [ns]
float64 last_sync_age        # [s] local_time_now - last_sync_time as seen by the lidar (NaN if never synced)
float64 time_since_sync_update  # [s] host time since last_sync_time was last seen to change (-1 if never)
uint64 sync_updates          # how many times last_sync_time was seen to change
uint64 query_failures

# ---------------- (b) Streamed data packets ----------------
bool packets_valid           # at least one point/IMU packet has been received
float64 packet_age           # [s] host time since the last packet
uint8 packet_time_type       # one of SYNC_* (as stamped in the packet header)
uint64 packet_timestamp      # lidar timestamp of the last packet [ns]
# [s] last packet timestamp minus host system clock at receipt. Close to 0 (plus network latency)
# when the lidar and this host share the same time base. ~37 s indicates a TAI/UTC mismatch.
float64 lidar_minus_host
uint64 packets_synced
uint64 packets_unsynced
