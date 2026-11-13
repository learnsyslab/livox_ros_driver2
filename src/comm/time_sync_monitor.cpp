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

#include "time_sync_monitor.h"

#include <chrono>
#include <cstring>

#include "livox_lidar_api.h"

namespace livox_ros {

TimeSyncMonitor& TimeSyncMonitor::GetInstance() {
  static TimeSyncMonitor instance;
  return instance;
}

int64_t TimeSyncMonitor::HostNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

void TimeSyncMonitor::AddLidar(uint32_t handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  states_[handle].handle = handle;
}

void TimeSyncMonitor::OnPacket(uint32_t handle, uint8_t time_type, uint64_t timestamp_ns) {
  const int64_t now = HostNowNs();
  std::lock_guard<std::mutex> lock(mutex_);
  TimeSyncState& s = states_[handle];
  s.handle = handle;
  s.packet_valid = true;
  s.packet_host_ns = now;
  s.packet_time_type = time_type;
  s.packet_timestamp = timestamp_ns;
  if (time_type == 0) {
    ++s.packets_unsynced;
  } else {
    ++s.packets_synced;
  }
}

void TimeSyncMonitor::RequestStatus() {
  std::vector<uint32_t> handles;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& kv : states_) {
      handles.push_back(kv.first);
    }
  }
  for (uint32_t handle : handles) {
    if (QueryLivoxLidarInternalInfo(handle, &TimeSyncMonitor::OnQueryResponse, this) !=
        kLivoxLidarStatusSuccess) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++states_[handle].query_failures;
    }
  }
}

std::vector<TimeSyncState> TimeSyncMonitor::Snapshot() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TimeSyncState> out;
  out.reserve(states_.size());
  for (const auto& kv : states_) {
    out.push_back(kv.second);
  }
  return out;
}

void TimeSyncMonitor::OnQueryResponse(livox_status status, uint32_t handle,
                                      LivoxLidarDiagInternalInfoResponse* response,
                                      void* client_data) {
  auto* self = static_cast<TimeSyncMonitor*>(client_data);
  if (self == nullptr) {
    return;
  }
  const int64_t now = HostNowNs();

  std::lock_guard<std::mutex> lock(self->mutex_);
  TimeSyncState& s = self->states_[handle];
  s.handle = handle;
  if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
    ++s.query_failures;
    return;
  }

  bool got_sync_type = false;
  bool got_last_sync = false;
  uint64_t last_sync_time = s.last_sync_time;

  // The response payload is a packed list of {uint16 key, uint16 length, uint8 value[length]}.
  uint32_t off = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    const auto* kv = reinterpret_cast<const LivoxLidarKeyValueParam*>(&response->data[off]);
    const uint16_t key = kv->key;
    const uint16_t len = kv->length;
    const uint8_t* value = kv->value;

    switch (key) {
      case kKeyLocalTimeNow:
        if (len >= sizeof(uint64_t)) std::memcpy(&s.local_time_now, value, sizeof(uint64_t));
        break;
      case kKeyLastSyncTime:
        if (len >= sizeof(uint64_t)) {
          std::memcpy(&last_sync_time, value, sizeof(uint64_t));
          got_last_sync = true;
        }
        break;
      case kKeyTimeOffset:
        if (len >= sizeof(int64_t)) std::memcpy(&s.time_offset, value, sizeof(int64_t));
        break;
      case kKeyTimeSyncType:
        if (len >= sizeof(uint8_t)) {
          s.time_sync_type = value[0];
          got_sync_type = true;
        }
        break;
      default:
        break;
    }
    off += 2 * sizeof(uint16_t) + len;
  }

  if (!got_sync_type && !got_last_sync) {
    // This firmware/device type does not report time sync keys.
    ++s.query_failures;
    return;
  }

  // last_sync_time is the master timestamp carried by the most recent sync message.
  // If it changes between two queries, the lidar is actively receiving sync messages.
  if (got_last_sync) {
    if (s.status_valid && last_sync_time != 0 && last_sync_time != s.last_sync_time) {
      s.last_sync_change_host_ns = now;
      ++s.sync_updates;
    }
    s.last_sync_time = last_sync_time;
  }
  s.status_valid = true;
  s.status_host_ns = now;
}

}  // namespace livox_ros
