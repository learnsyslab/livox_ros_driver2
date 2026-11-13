//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_ROS_DRIVER_TIME_SYNC_MONITOR_H_
#define LIVOX_ROS_DRIVER_TIME_SYNC_MONITOR_H_

#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include "livox_lidar_def.h"

namespace livox_ros {

/**
 * Raw time synchronization observations for one lidar.
 * All *_host_ns fields are host CLOCK_REALTIME (std::chrono::system_clock) in ns.
 */
struct TimeSyncState {
  uint32_t handle = 0;

  // ---- From the lidar's internal status (QueryLivoxLidarInternalInfo) ----
  bool status_valid = false;
  int64_t status_host_ns = 0;        // host time the last status response arrived
  uint8_t time_sync_type = 0;        // key 0x800C: 0 none, 1 PTP (IEEE 1588v2), 2 GPS
  uint64_t local_time_now = 0;       // key 0x8009: lidar local time [ns]
  uint64_t last_sync_time = 0;       // key 0x800A: master time t1 of the last PTP Follow_Up / GPS PPS [ns]
  int64_t time_offset = 0;           // key 0x800B: lidar local time - sync source time [ns]
  int64_t last_sync_change_host_ns = 0;  // host time at which last_sync_time was last seen to change
  uint64_t sync_updates = 0;         // number of times last_sync_time was seen to change
  uint64_t query_failures = 0;

  // ---- From point cloud / IMU packet headers ----
  bool packet_valid = false;
  int64_t packet_host_ns = 0;        // host time the last packet arrived
  uint8_t packet_time_type = 0;      // 0 no sync (time since power-on), 1 PTP/gPTP, 2 GPS
  uint64_t packet_timestamp = 0;     // lidar timestamp of the last packet [ns]
  uint64_t packets_synced = 0;
  uint64_t packets_unsynced = 0;
};

/**
 * Collects time synchronization information for all connected Livox lidars.
 * Thread-safe: OnPacket() is called from the SDK data thread, query responses arrive
 * on the SDK command thread, Snapshot()/RequestStatus() are called from the ROS timer.
 */
class TimeSyncMonitor {
 public:
  static TimeSyncMonitor& GetInstance();

  /** Register a lidar so that it is polled even before any data arrives. */
  void AddLidar(uint32_t handle);

  /** Record the timestamp type/value of one point cloud or IMU packet. */
  void OnPacket(uint32_t handle, uint8_t time_type, uint64_t timestamp_ns);

  /** Asynchronously query the time sync status of every known lidar. */
  void RequestStatus();

  std::vector<TimeSyncState> Snapshot();

  static int64_t HostNowNs();

 private:
  TimeSyncMonitor() = default;
  TimeSyncMonitor(const TimeSyncMonitor&) = delete;
  TimeSyncMonitor& operator=(const TimeSyncMonitor&) = delete;

  static void OnQueryResponse(livox_status status, uint32_t handle,
                              LivoxLidarDiagInternalInfoResponse* response, void* client_data);

  std::mutex mutex_;
  std::map<uint32_t, TimeSyncState> states_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_TIME_SYNC_MONITOR_H_
