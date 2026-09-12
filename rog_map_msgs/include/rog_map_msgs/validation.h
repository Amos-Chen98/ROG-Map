#pragma once
#include <rog_map_msgs/LocalMap.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rog_map_msgs {
inline bool bit(const std::vector<uint8_t>& data, size_t index) {
  return (data[index / 8] & (uint8_t(1) << (index % 8))) != 0;
}
inline void setBit(std::vector<uint8_t>& data, size_t index) {
  data[index / 8] |= uint8_t(1) << (index % 8);
}
inline size_t validate(const LocalMap& map) {
  if (map.header.frame_id.empty() || !map.epoch || !map.version ||
      !std::isfinite(map.resolution) || map.resolution <= 0 ||
      !std::isfinite(map.origin.x) || !std::isfinite(map.origin.y) || !std::isfinite(map.origin.z))
    throw std::invalid_argument("Invalid local map metadata");
  size_t count = 1;
  for (auto size : map.size) {
    // Bound allocation and the signed integer indexing used by both consumers.
    if (!size || count > 16000000u / size)
      throw std::invalid_argument("Invalid or excessive local map dimensions");
    count *= size;
  }
  const size_t bytes = (count + 7) / 8;
  if (map.occupied_bits.size() != bytes || map.inflated_bits.size() != bytes)
    throw std::invalid_argument("Invalid local map bitmap length");
  if (count % 8) {
    const uint8_t mask = uint8_t(0xffu << (count % 8));
    if ((map.occupied_bits.back() | map.inflated_bits.back()) & mask)
      throw std::invalid_argument("Nonzero local map padding bits");
  }
  for (size_t i = 0; i < bytes; ++i)
    if (map.occupied_bits[i] & ~map.inflated_bits[i])
      throw std::invalid_argument("Inflation must contain raw occupancy");
  return count;
}
}
