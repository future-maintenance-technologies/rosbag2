// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
#define ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include "rosbag2_storage/visibility_control.hpp"
#include "rosbag2_storage/yaml.hpp"

namespace rosbag2_storage
{

struct StorageOptions
{
public:
  std::string uri;
  std::string storage_id;

  // The maximum size a bagfile can be, in bytes, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_size = 0;

  // The maximum duration a bagfile can be, in seconds, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_duration = 0;

  // Maximum number of bag files to retain before deleting the oldest.
  // A value of 0 disables deletion (unlimited files).
  // This feature is only available when the bag split is active.
  // Requires --max-bag-size or --max-bag-duration to be set or usage of the bag split via
  // direct recorder API or service calls.
  uint64_t max_bag_files = 0;

  // The cache size. Indicates how many messages can maximally be held in cache before these being
  // written to disk. Works together with max_cache_duration bound if set.
  // A value of 0 disables size-based caching and every write happens directly to disk if
  // max_cache_duration is also set to 0.
  uint64_t max_cache_size = 0;

  // Maximum cache duration in seconds. Used for time-limited buffering (applies to both snapshot
  // mode and regular caching). A value of 0 indicates that buffering will be limited by the
  // max_cache_size only. When greater than 0, the cache buffer maintains messages within
  // this time window and drops newer messages if cache overflow happened.
  // Works together with max_cache_size bound if set.
  uint32_t max_cache_duration = 0;

  // Preset storage configuration. Preset settings can be overriden with
  // corresponding settings specified through storage_config_uri file
  std::string storage_preset_profile = "";

  // Storage specific configuration file.
  // Defaults to empty string.
  std::string storage_config_uri = "";

  // Enable snapshot mode.
  // Defaults to disabled.
  bool snapshot_mode = false;

  // Defer a duration-triggered split until an H.264 keyframe has been seen, so
  // each split file begins on a decodable keyframe per video stream.
  // Defaults to disabled (exact upstream split behaviour).
  bool split_on_keyframe = false;

  // How many seconds before a duration split is due to start buffering incoming messages
  // instead of writing them immediately, so the whole upcoming keyframe cluster is visible
  // before any of it is committed — this is what lets the writer place the file cut exactly
  // before the cluster with zero dropped frames and zero duplicated messages, regardless of
  // exactly which message happens to cross the duration deadline first.
  // Must be at least as long as the video encoders' forced-keyframe recurrence period; only
  // used when split_on_keyframe is enabled.
  //
  // NOTE: this value is coupled to whatever recording node forces the keyframes (e.g. ARIIS's
  // image_compressor_node `keyframe_interval_sec` param) but there is no automatic enforcement
  // of that relationship on either side. If the two drift out of sync (this value ends up
  // shorter than the actual recurrence period), the look-ahead buffer simply won't contain a
  // full cluster as often — no crash, just a silent partial regression back toward the old
  // per-stream "wait for the next keyframe" behaviour. Keep them in sync by hand.
  double keyframe_lookback_sec = 1.0;

  // Start and end time for cutting. Used in the writers to limit the range of stored messages.
  // As well as in the "ros2 bag convert" CLI aka "bag_rewrite" utility to limit the range of the
  // reading and writing messages.
  int64_t start_time_ns = -1;
  int64_t end_time_ns = -1;

  // Stores the custom data
  std::unordered_map<std::string, std::string> custom_data{};
};

}  // namespace rosbag2_storage

namespace YAML
{
template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rosbag2_storage::StorageOptions>
{
  static Node encode(const rosbag2_storage::StorageOptions & storage_options);
  static bool decode(const Node & node, rosbag2_storage::StorageOptions & storage_options);
};
}  // namespace YAML

#endif  // ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
