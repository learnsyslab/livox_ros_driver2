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

// Periodically publishes the PTP/GPS time synchronization status of every connected
// lidar on "livox/time_sync_status" and logs state transitions.

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <string>

#include "driver_node.h"
#include "comm/comm.h"
#include "comm/time_sync_monitor.h"

namespace livox_ros {

namespace {

using TimeSyncStatusMsg = livox_ros_driver2::msg::TimeSyncStatus;

constexpr double kNsToSec = 1e-9;

const char* SyncTypeName(uint8_t type) {
  switch (type) {
    case TimeSyncStatusMsg::SYNC_NONE: return "none";
    case TimeSyncStatusMsg::SYNC_PTP:  return "PTP";
    case TimeSyncStatusMsg::SYNC_GPS:  return "GPS";
    default:                           return "unknown";
  }
}

std::string Format(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
std::string Format(const char* fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return buf;
}

}  // namespace

void DriverNode::InitTimeSyncMonitor() {
  const double period = this->declare_parameter("time_sync_status_period", 1.0);
  time_sync_timeout_ = this->declare_parameter("time_sync_timeout", 3.0);
  this->get_parameter("frame_id", time_sync_frame_id_);

  if (period <= 0.0) {
    DRIVER_INFO(*this, "Time sync status monitoring disabled (time_sync_status_period <= 0).");
    return;
  }
  if (time_sync_timeout_ < period) {
    DRIVER_WARN(*this, "time_sync_timeout (%.2f s) < time_sync_status_period (%.2f s); using %.2f s.",
                time_sync_timeout_, period, 2.0 * period);
    time_sync_timeout_ = 2.0 * period;
  }

  time_sync_pub_ = this->create_publisher<TimeSyncStatusMsg>("livox/time_sync_status", rclcpp::QoS(10));
  time_sync_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period), [this]() { PublishTimeSyncStatus(); });
  DRIVER_INFO(*this, "Publishing lidar time sync status on '%s' every %.2f s (sync timeout %.2f s).",
              time_sync_pub_->get_topic_name(), period, time_sync_timeout_);
}

void DriverNode::PublishTimeSyncStatus() {
  const int64_t now_ns = TimeSyncMonitor::HostNowNs();
  const auto states = TimeSyncMonitor::GetInstance().Snapshot();

  for (const auto& s : states) {
    TimeSyncStatusMsg msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = time_sync_frame_id_;
    msg.lidar_ip = IpNumToString(s.handle);

    // ---- (a) lidar-reported status ----
    msg.status_valid = s.status_valid;
    msg.status_age = s.status_valid ? (now_ns - s.status_host_ns) * kNsToSec : -1.0;
    msg.time_sync_type = s.time_sync_type;
    msg.local_time_now = s.local_time_now;
    msg.last_sync_time = s.last_sync_time;
    msg.time_offset = s.time_offset;
    msg.last_sync_age = (s.status_valid && s.last_sync_time != 0)
        ? static_cast<int64_t>(s.local_time_now - s.last_sync_time) * kNsToSec
        : std::numeric_limits<double>::quiet_NaN();
    msg.time_since_sync_update = s.last_sync_change_host_ns > 0
        ? (now_ns - s.last_sync_change_host_ns) * kNsToSec : -1.0;
    msg.sync_updates = s.sync_updates;
    msg.query_failures = s.query_failures;

    // ---- (b) streamed data packets ----
    msg.packets_valid = s.packet_valid;
    msg.packet_age = s.packet_valid ? (now_ns - s.packet_host_ns) * kNsToSec : -1.0;
    msg.packet_time_type = s.packet_time_type;
    msg.packet_timestamp = s.packet_timestamp;
    msg.lidar_minus_host = s.packet_valid
        ? (static_cast<int64_t>(s.packet_timestamp) - s.packet_host_ns) * kNsToSec
        : std::numeric_limits<double>::quiet_NaN();
    msg.packets_synced = s.packets_synced;
    msg.packets_unsynced = s.packets_unsynced;

    // ---- verdict ----
    msg.receiving_sync_messages = msg.time_since_sync_update >= 0.0 &&
                                  msg.time_since_sync_update <= time_sync_timeout_;
    const bool status_fresh = s.status_valid && msg.status_age <= time_sync_timeout_;
    const bool packets_fresh = s.packet_valid && msg.packet_age <= time_sync_timeout_;
    const bool packets_ok = !packets_fresh || s.packet_time_type != TimeSyncStatusMsg::SYNC_NONE;

    std::string state;
    if (!status_fresh && !packets_fresh) {
      state = "NO_DATA";
      msg.summary = s.status_valid || s.packet_valid
          ? "no status response or data packet from lidar recently"
          : "waiting for lidar status / data";
    } else if (!status_fresh) {
      // Status query not available: fall back to what the data packets say.
      state = s.packet_time_type == TimeSyncStatusMsg::SYNC_NONE ? "FREE_RUNNING" : "SYNCED_PACKETS_ONLY";
      msg.summary = Format("status query unavailable (%lu failures); packets stamped with '%s' time, "
                           "lidar-host %.6f s", static_cast<unsigned long>(s.query_failures),
                           SyncTypeName(s.packet_time_type), msg.lidar_minus_host);
    } else if (s.time_sync_type == TimeSyncStatusMsg::SYNC_NONE) {
      state = "FREE_RUNNING";
      msg.summary = "NOT synchronized: lidar has no time sync source (no PTP master seen); "
                    "timestamps are time since power-on";
    } else if (!msg.receiving_sync_messages) {
      state = Format("%s_STALE", SyncTypeName(s.time_sync_type));
      msg.summary = msg.time_since_sync_update < 0.0
          ? Format("lidar reports %s sync but no new sync message observed yet "
                   "(last_sync_age %.3f s, offset %.3f us)",
                   SyncTypeName(s.time_sync_type), msg.last_sync_age, s.time_offset * 1e-3)
          : Format("lidar reports %s sync but received NO new sync message for %.1f s "
                   "(offset %.3f us)", SyncTypeName(s.time_sync_type),
                   msg.time_since_sync_update, s.time_offset * 1e-3);
    } else if (!packets_ok) {
      state = Format("%s_PACKETS_UNSYNCED", SyncTypeName(s.time_sync_type));
      msg.summary = Format("lidar receives %s sync messages but data packets are still stamped "
                           "with unsynchronized time", SyncTypeName(s.time_sync_type));
    } else {
      state = Format("%s_LOCKED", SyncTypeName(s.time_sync_type));
      msg.synchronized = true;
      msg.summary = Format("%s locked: offset %.3f us, last sync %.3f s ago, lidar-host %.6f s",
                           SyncTypeName(s.time_sync_type), s.time_offset * 1e-3,
                           msg.last_sync_age, msg.lidar_minus_host);
    }

    auto& last_state = time_sync_last_state_[s.handle];
    if (state != last_state) {
      if (msg.synchronized) {
        DRIVER_INFO(*this, "[%s] time sync: %s -> %s: %s", msg.lidar_ip.c_str(),
                    last_state.empty() ? "START" : last_state.c_str(), state.c_str(), msg.summary.c_str());
      } else {
        DRIVER_WARN(*this, "[%s] time sync: %s -> %s: %s", msg.lidar_ip.c_str(),
                    last_state.empty() ? "START" : last_state.c_str(), state.c_str(), msg.summary.c_str());
      }
      last_state = state;
    }

    time_sync_pub_->publish(msg);
  }

  // Ask for fresh status; responses arrive asynchronously and are used in the next cycle.
  TimeSyncMonitor::GetInstance().RequestStatus();
}

}  // namespace livox_ros
