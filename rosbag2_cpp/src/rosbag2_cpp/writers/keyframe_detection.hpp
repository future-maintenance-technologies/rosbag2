// Copyright 2026 Future Maintenance Technologies.
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

#ifndef ROSBAG2_CPP__WRITERS__KEYFRAME_DETECTION_HPP_
#define ROSBAG2_CPP__WRITERS__KEYFRAME_DETECTION_HPP_

#include <cstddef>
#include <cstdint>

// Self-contained H.264 keyframe detection for foxglove_msgs/msg/CompressedVideo.
// Header-only and free of ROS/rcutils dependencies so it can be unit-tested in
// isolation and included directly by the writer. See keyframe-aware split.

namespace rosbag2_cpp
{
namespace writers
{

/// True if the Annex-B byte stream contains an IDR slice (a keyframe).
/// Scans for start codes (00 00 01 / 00 00 00 01); a NAL is an IDR when its
/// type (nal_header & 0x1F) == 5. Early-outs on the first IDR.
inline bool nal_stream_has_idr(const uint8_t * d, size_t len)
{
  if (d == nullptr) {
    return false;
  }
  for (size_t i = 0; i + 3 < len; ++i) {
    const bool sc3 = (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1);
    const bool sc4 = (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 0 && d[i + 3] == 1);
    if (sc3 || sc4) {
      const size_t h = i + (sc4 ? 4 : 3);
      if (h < len && (d[h] & 0x1F) == 5) {
        return true;
      }
      i = h;  // skip past this start code
    }
  }
  return false;
}

/// True if a CDR-serialized foxglove_msgs/msg/CompressedVideo carries an IDR.
/// Skips over the fixed/preceding fields to locate the `data` blob without a
/// full ROS deserialization, then NAL-scans it. Every read is bounds-checked;
/// a malformed or truncated buffer returns false rather than reading OOB.
///
/// CDR layout (little- or big-endian per the encapsulation header):
///   [0..3] encapsulation header (alignment origin resets after it)
///   Time timestamp : int32 sec, uint32 nanosec   (8 bytes)
///   string frame_id: uint32 len (4-aligned) + len bytes (incl. NUL)
///   uint8[] data   : uint32 len (4-aligned) + len bytes   <-- scanned
///   string format  : ignored
inline bool compressed_video_has_idr(const uint8_t * cdr, size_t n)
{
  if (cdr == nullptr || n < 4) {
    return false;
  }
  // Representation id (first two octets, big-endian): odd value == little-endian.
  const bool little_endian = (cdr[1] & 0x01) != 0;
  auto rd_u32 = [&](size_t off) -> uint32_t {
      if (little_endian) {
        return static_cast<uint32_t>(cdr[off]) |
               (static_cast<uint32_t>(cdr[off + 1]) << 8) |
               (static_cast<uint32_t>(cdr[off + 2]) << 16) |
               (static_cast<uint32_t>(cdr[off + 3]) << 24);
      }
      return (static_cast<uint32_t>(cdr[off]) << 24) |
             (static_cast<uint32_t>(cdr[off + 1]) << 16) |
             (static_cast<uint32_t>(cdr[off + 2]) << 8) |
             static_cast<uint32_t>(cdr[off + 3]);
    };

  size_t p = 4;  // skip encapsulation header; alignment is measured from here
  auto align4 = [&]() {p += (4 - ((p - 4) & 3)) & 3;};

  p += 8;  // Time.sec + Time.nanosec
  align4();
  if (p + 4 > n) {
    return false;
  }
  const uint32_t fid_len = rd_u32(p);
  p += 4;
  if (fid_len > n - p) {  // avoids p + fid_len overflow
    return false;
  }
  p += fid_len;  // frame_id bytes (length includes the NUL)

  align4();
  if (p + 4 > n) {
    return false;
  }
  const uint32_t data_len = rd_u32(p);
  p += 4;
  if (data_len > n - p) {
    return false;
  }
  return nal_stream_has_idr(cdr + p, data_len);
}

}  // namespace writers
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__WRITERS__KEYFRAME_DETECTION_HPP_
