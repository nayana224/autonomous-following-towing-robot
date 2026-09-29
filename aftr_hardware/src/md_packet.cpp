// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#include "mdbot_hardware/md_packet.hpp"

namespace mdbot
{

MDPacket::MDPacket() {}
MDPacket::~MDPacket() {}

// Use the motor protocol's 8-bit two's-complement checksum.
uint8_t MDPacket::calcCheckSum(const std::vector<uint8_t>& packet)
{
  uint8_t sum = 0;

  // Callers pass the packet body before appending the checksum byte.
  for (size_t i = 0; i < packet.size(); i++)
  {
    sum += packet[i];
  }
  return ~(sum) + 1;
}

std::vector<uint8_t> MDPacket::createPacket(PID pid, const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> packet;

  packet.emplace_back(HEADER_RMID);
  packet.emplace_back(HEADER_TMID);
  packet.emplace_back(DRIVER_ID);

  packet.emplace_back(static_cast<uint8_t>(pid));

  packet.emplace_back(static_cast<uint8_t>(data.size()));

  packet.insert(packet.end(), data.begin(), data.end());

  packet.emplace_back(calcCheckSum(packet));

  return packet;
}

std::vector<std::vector<uint8_t>> MDPacket::parseBuffer(std::vector<uint8_t>& buffer)
{
  std::vector<std::vector<uint8_t>> valid_packets;

  // A packet needs at least six bytes before payload validation.
  while(buffer.size() >= 6)
  {
    // Response packets reverse the RMID/TMID sender and receiver addresses.
    // HEADER_TMID = 184(PC) -> RMID, HEADER_RMID = 183(MD400T) -> TMID
    if (buffer[0] == HEADER_TMID && buffer[1] == HEADER_RMID)
    {
      uint8_t data_len = buffer[4];
      size_t total_packet_len = 6 + data_len;

      // Wait until the entire packet is buffered.
      if (buffer.size() < total_packet_len)
      {
        break;
      }

      std::vector<uint8_t> temp_packet(buffer.begin(), buffer.begin() + total_packet_len);

      uint8_t received_chk = temp_packet.back();
      temp_packet.pop_back();

      if (calcCheckSum(temp_packet) == received_chk)
      {
        temp_packet.emplace_back(received_chk);
        valid_packets.emplace_back(temp_packet);

        buffer.erase(buffer.begin(), buffer.begin() + total_packet_len);
      }
      else
      {
        // Discard a packet when its checksum fails.
        buffer.erase(buffer.begin());
      }
    }
    else
    {
      // Discard one byte and resynchronize after a header mismatch.
      buffer.erase(buffer.begin());
    }
  }

  return valid_packets;
}

uint8_t MDPacket::getPID(const std::vector<uint8_t>& packet)
{
  if (packet.size() > 3) return packet[3];
  return 0;
}

std::vector<uint8_t> MDPacket::getData(const std::vector<uint8_t>& packet)
{
  std::vector<uint8_t> data;
  if (packet.size() < 6) return data;

  uint8_t len = packet[4];
  if (packet.size() < static_cast<size_t>(6 + len)) return data;

  data.assign(packet.begin() + 5, packet.begin() + 5 + len);
  return data;
}

} // namespace mdbot