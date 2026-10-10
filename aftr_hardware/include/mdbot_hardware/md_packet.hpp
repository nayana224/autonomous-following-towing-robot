// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#pragma once

#include <cstdint>
#include <vector>

#include "mdbot_hardware/md_defines.hpp"

namespace mdbot
{

/// Build and parse MD motor protocol packets.
class MDPacket
{
public:
  MDPacket();
  ~MDPacket();

  /// Build a motor command packet with its checksum.
  std::vector<uint8_t>createPacket(PID pid, const std::vector<uint8_t>& data = {});

  /// Extract complete checksum-valid packets from the receive buffer.
  std::vector<std::vector<uint8_t>> parseBuffer(std::vector<uint8_t>& buffer);

  /// Read the PID field from a valid packet.
  uint8_t getPID(const std::vector<uint8_t>& packet);

  /// Extract the payload from a valid packet.
  std::vector<uint8_t> getData(const std::vector<uint8_t>& packet);

private:
  /// Return the 8-bit two's-complement checksum for the packet body.
  uint8_t calcCheckSum(const std::vector<uint8_t>& packet_body);
};

} // namespace mdbot

