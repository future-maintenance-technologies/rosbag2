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

#include <gmock/gmock.h>

#include <cstdint>
#include <vector>

#include "rosbag2_cpp/writers/keyframe_detection.hpp"

using rosbag2_cpp::writers::nal_stream_has_idr;
using rosbag2_cpp::writers::compressed_video_has_idr;

namespace
{
// NAL with a 4-byte start code and the given type (low 5 bits of the header).
void append_nal(std::vector<uint8_t> & v, uint8_t type, bool three_byte = false)
{
  if (!three_byte) {
    v.push_back(0);
  }
  v.insert(v.end(), {0, 0, 1});
  v.push_back(type & 0x1F);  // nal_ref_idc 0 for brevity; type in low bits
  v.push_back(0xAB);
}

// Build a minimal little-endian CDR CompressedVideo buffer wrapping `payload`.
std::vector<uint8_t> make_cdr(
  const std::vector<uint8_t> & payload,
  const std::string & frame_id = "cam")
{
  std::vector<uint8_t> b = {0x00, 0x01, 0x00, 0x00};  // CDR_LE encapsulation header
  auto push_u32 = [&](uint32_t x) {
      b.push_back(x & 0xFF); b.push_back((x >> 8) & 0xFF);
      b.push_back((x >> 16) & 0xFF); b.push_back((x >> 24) & 0xFF);
    };
  auto align4 = [&]() {while (((b.size() - 4) & 3) != 0) {b.push_back(0);}};

  push_u32(0);  // Time.sec
  push_u32(0);  // Time.nanosec
  align4();
  const uint32_t fid_len = static_cast<uint32_t>(frame_id.size()) + 1;  // incl NUL
  push_u32(fid_len);
  b.insert(b.end(), frame_id.begin(), frame_id.end());
  b.push_back(0);
  align4();
  push_u32(static_cast<uint32_t>(payload.size()));
  b.insert(b.end(), payload.begin(), payload.end());
  return b;
}
}  // namespace

TEST(KeyframeDetection, idr_present_four_byte_start_code)
{
  std::vector<uint8_t> s;
  append_nal(s, 7);  // SPS
  append_nal(s, 8);  // PPS
  append_nal(s, 5);  // IDR
  EXPECT_TRUE(nal_stream_has_idr(s.data(), s.size()));
}

TEST(KeyframeDetection, idr_present_three_byte_start_code)
{
  std::vector<uint8_t> s;
  append_nal(s, 1, /*three_byte=*/true);
  append_nal(s, 5, /*three_byte=*/true);
  EXPECT_TRUE(nal_stream_has_idr(s.data(), s.size()));
}

TEST(KeyframeDetection, delta_frame_only_no_idr)
{
  std::vector<uint8_t> s;
  append_nal(s, 1);  // non-IDR slice
  append_nal(s, 1);
  EXPECT_FALSE(nal_stream_has_idr(s.data(), s.size()));
}

TEST(KeyframeDetection, sps_pps_but_no_idr)
{
  std::vector<uint8_t> s;
  append_nal(s, 7);
  append_nal(s, 8);
  append_nal(s, 1);
  EXPECT_FALSE(nal_stream_has_idr(s.data(), s.size()));
}

TEST(KeyframeDetection, empty_and_null)
{
  EXPECT_FALSE(nal_stream_has_idr(nullptr, 0));
  std::vector<uint8_t> tiny = {0, 0};
  EXPECT_FALSE(nal_stream_has_idr(tiny.data(), tiny.size()));
}

TEST(KeyframeDetection, cdr_keyframe)
{
  std::vector<uint8_t> s;
  append_nal(s, 7);
  append_nal(s, 5);
  const auto cdr = make_cdr(s);
  EXPECT_TRUE(compressed_video_has_idr(cdr.data(), cdr.size()));
}

TEST(KeyframeDetection, cdr_delta_frame)
{
  std::vector<uint8_t> s;
  append_nal(s, 1);
  const auto cdr = make_cdr(s);
  EXPECT_FALSE(compressed_video_has_idr(cdr.data(), cdr.size()));
}

TEST(KeyframeDetection, cdr_truncated_returns_false_no_oob)
{
  std::vector<uint8_t> s;
  append_nal(s, 5);
  const auto cdr = make_cdr(s);
  // Every truncation length must be handled without reading out of bounds.
  for (size_t len = 0; len < cdr.size(); ++len) {
    EXPECT_FALSE(compressed_video_has_idr(cdr.data(), len)) << "len=" << len;
  }
  EXPECT_TRUE(compressed_video_has_idr(cdr.data(), cdr.size()));
}
