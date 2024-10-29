/**
 *
 *  \file
 *  \brief      Implementation of Comms class methods to handle reading and
 *              writing to the UM7 serial interface.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "um7/comms.h"

#include <arpa/inet.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <chrono>
// #include <deque>

#include "ros/console.h"
#include "serial/serial.h"
#include "um7/registers.h"

namespace um7
{

const uint8_t Comms::PACKET_HAS_DATA = 1 << 7;
const uint8_t Comms::PACKET_IS_BATCH = 1 << 6;
const uint8_t Comms::PACKET_BATCH_LENGTH_MASK = 0x0F;
const uint8_t Comms::PACKET_BATCH_LENGTH_OFFSET = 2;

int16_t Comms::receive(Registers* registers = NULL)
{
  // Search the serial stream for a start-of-packet sequence.
  try
  {
    size_t available = serial_->available();
    if (available > 1024)
    {
      ROS_WARN_STREAM("Serial read buffer is " << available << ", now flushing in an attempt to catch up.");
      serial_->flushInput();
    }

    uint8_t type, address;
    std::string snp;

    serial_->readline(snp, 96, "snp");
    if (!boost::algorithm::ends_with(snp, "snp")) {
      throw SerialTimeout();
    }
    if (snp.length() > 3)
    {
      // ROS_WARN_STREAM_COND(!first_spin_,
      //   "Discarded " << 5 + snp.length() - 3 << " junk byte(s) preceeding packet.");
    }
    if (serial_->read(&type, 1) != 1) {
      throw SerialTimeout();
    }
    if (serial_->read(&address, 1) != 1) {
      throw SerialTimeout();
    }


    first_spin_ = false;

    uint16_t checksum_calculated = 's' + 'n' + 'p' + type + address;
    std::string data;
    if (type & PACKET_HAS_DATA)
    {
      uint8_t data_length = 1;
      if (type & PACKET_IS_BATCH)
      {
        data_length = (type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        ROS_DEBUG("Received packet %02x with batched (%d) data.", address, data_length);
      }
      else
      {
        ROS_DEBUG("Received packet %02x with non-batched data.", address);
      }

      // Read data bytes initially into a buffer so that we can compute the checksum.
      if (serial_->read(data, data_length * 4) != data_length * 4) {
        throw SerialTimeout();
      }
      BOOST_FOREACH(uint8_t ch, data)
      {
        checksum_calculated += ch;
      }
    }
    else
    {
      ROS_INFO("Received packet %02x without data.", address);
    }

    // Compare computed checksum with transmitted value.
    uint16_t checksum_transmitted;
    if (serial_->read(reinterpret_cast<uint8_t*>(&checksum_transmitted), 2) != 2)
    {
      throw SerialTimeout();
    }
    checksum_transmitted = ntohs(checksum_transmitted);
    if (checksum_transmitted != checksum_calculated)
    {
      throw BadChecksum();
    }

    // Copy data from checksum buffer into registers, if specified.
    // Note that byte-order correction (as necessary) happens at access-time
    if ((data.length() > 0) && registers)
    {
      registers->write_raw(address, data);
      // if (address == DREG_HEALTH){
      //   uint32_t num =  (uint32_t)data[0] << 24 |
      //                   (uint32_t)data[1] << 16 |
      //                   (uint32_t)data[2] << 8  |
      //                   (uint32_t)data[3];

      //   ROS_ERROR("*** HEALTH L: %d, num: %08x, get: %08x", (int)data.length(),  num, registers->health.get(0));
      //   if(bool(num & HEALTH_COM_OVERFLOW)) {
      //     ROS_ERROR("OVERFLOW");
      //   } 
      //   else {
      //     ROS_WARN("No Overflow");
      //   }
      // }
    }

    // Successful packet read, return address byte.
    return address;
  }
  catch(const SerialTimeout& e)
  {
    ROS_WARN("Timed out waiting for packet from device.");
  }
  catch(const BadChecksum& e)
  {
    ROS_WARN("Discarding packet due to bad checksum.");
  }
  return -1;
}


// This will block and read serial until timeout. Will not flush buffer.
// Re-implemented readline here, so that it is inside the ack timeout.
// This will write the data payload of the packet onto data_sptr.
bool Comms::receiveSpecificPacket(const Accessor_& a, std::string* data_sptr, uint8_t& packet_type, const unsigned int& _wait_ack_timeout_ms)
{
  // uint8_t type, address;
  std::string snp;
  
  const uint8_t preamble[3]{'s', 'n', 'p'};
  // NOTE: Max packet length is 7+(4*16) = 71 bytes
  uint8_t pbuf[80];
  unsigned int idx_pbuf=0, idx_chksum=5;
  uint16_t checksum_calculated;
  uint16_t checksum_transmitted;
  uint8_t& type = pbuf[3];
  uint8_t& address = pbuf[4];
  uint32_t data_byte_length=0;
  

    
  auto begin_time = std::chrono::steady_clock::now();

  // auto now_time = begin_time;
  // ROS_WARN("@@@@@@@@ avail: %d", (int)serial_->available());
  while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count() < _wait_ack_timeout_ms)
  {
    // If you go through the body of this while loop WITHOUT skipping (continue), you would have received the whole packet
    // Read the preamble, and check if they are valid
    while (idx_pbuf < 3) {
      if (serial_->read(&pbuf[idx_pbuf], 1) == 1) {
        // check if the byte we just read, is valid WRT the preamble
        if (pbuf[idx_pbuf] == preamble[idx_pbuf]) {
          idx_pbuf++;
        }
        else {
          goto redo_packet;
        }
      }
      else {
        throw SerialTimeout();
      }
    }

    // Read packet type and address only
    if (serial_->read(&pbuf[idx_pbuf], 2) == 2) {
      packet_type = pbuf[idx_pbuf];   // copy packet type
      idx_pbuf += 2;
    }
    else {
      throw SerialTimeout();
    }

    // Check if the address (idx=4) is the same
    if (pbuf[4] != a.index){
      // now_time = std::chrono::steady_clock::now();
      // ROS_WARN("@ Found packet has different address. Continue searching... elapsedms: %ld", std::chrono::duration_cast<std::chrono::milliseconds>(now_time - begin_time).count());
      goto redo_packet;
    }

    // We found the right address. Now read the rest of the packet and check if it pans out
    checksum_calculated = 's' + 'n' + 'p' + type + address;
    if (type & PACKET_HAS_DATA)
    {
      uint8_t data_length = 1;
      if (type & PACKET_IS_BATCH)
      {
        data_length = (type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        // ROS_DEBUG("Received packet %02x with batched (%d) data.", address, data_length);
      }
      else
      {
        // ROS_DEBUG("Received packet %02x with non-batched data.", address);
      }

      // Read data bytes initially into a buffer so that we can compute the checksum.
      data_byte_length = data_length * 4;
      if (serial_->read(&pbuf[idx_pbuf], data_byte_length) == data_byte_length) {
        idx_pbuf += data_byte_length;
      }
      else {
        // ROS_WARN("@ Reading DATA bytes timed out elapsedms: %ld", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count());
        throw SerialTimeout();
      }
      for(int i=5; i < 5+data_byte_length; i++)
      {
        checksum_calculated += pbuf[i];
      }
    }
    else
    {
      data_byte_length = 0;
      // ROS_WARN("Received packet %02x without data.", address);
    }

    // Compare computed checksum with transmitted value.
    idx_chksum = idx_pbuf;
    if (serial_->read(&pbuf[idx_pbuf], 2) == 2){
      idx_pbuf += 2;
    }
    else {
      throw SerialTimeout();
    }

    checksum_transmitted = ntohs(*reinterpret_cast<uint16_t*>(&pbuf[idx_chksum]));
    if (checksum_transmitted != checksum_calculated)
    {
      throw BadChecksum();
    }

    // Copy data from checksum buffer into the string data buffers, if specified.
    // Note that byte-order correction (as necessary) happens at access-time
    if ((data_byte_length > 0) && data_sptr)
    {
      *data_sptr = std::string(reinterpret_cast<char const*>(&pbuf[5]), data_byte_length);
    }



    // ROS_WARN("@@ Time_ms to receive ack: %ld", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count() );
    
    return true;

    // recycle goto
    recycle_rcvd_bytes:;
    // Recycle received bytes [UNFINISHED] if starting at index 4 (after preamble) if any of them satisfies the start of the preamble
    { // Code block to prevent "crosses initialization" errors
      bool found_s=false, found_n=false;
      int sc=0, nc=0, pc=0;
      for (int i=3; i<idx_pbuf; i++) {
        if (pbuf[i] == 's' && found_s==false)
        {
          found_s=true;
          sc++;
          // ROS_ERROR("## [S] found @ %d", i);
        }
        else if (found_s==true && pbuf[i]=='n') {
          found_n = true;
          nc++;
          // ROS_ERROR("## [N] found @ %d", i);
        }
        else if (found_n==true && pbuf[i]=='p') {
          // ROS_ERROR("## [P] found @ %d", i);
          // ROS_ERROR("##>> complete SNP preamble");
          pc++;
        }
        else if (found_s || found_n) {
          found_s = false;
          found_n = false;
          // ROS_ERROR("#### terminate find i: %d, s:%d, n:%d",i, (int)found_s, (int)found_n);
        }
      }

      if(sc>0 || nc>0 || pc>0){
        // ROS_ERROR("#### char counts: s=%d, n=%d, p=%d, ", sc, nc, pc);
      }
      else {
        // if (idx_pbuf >= 3){
        //   ROS_ERROR("#### NOthins");
        // }
      }
    }

    // Reset go to
    redo_packet:;
    idx_pbuf = 0;
    data_byte_length = 0;
    checksum_calculated = 0;
    checksum_transmitted = 0;
    idx_chksum=5;
  }

  // ROS_WARN("Timeout reached for reading an address: %ld", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count());
  return false;
}

std::string Comms::checksum(const std::string& s)
{
  uint16_t checksum = 0;
  BOOST_FOREACH(uint8_t ch, s)
  {
    checksum += ch;
  }
  checksum = htons(checksum);
  ROS_DEBUG("Computed checksum on string of length %zd as %04x.", s.length(), checksum);
  std::string out(2, 0);
  memcpy(&out[0], &checksum, 2);
  return out;
}

std::string Comms::message(uint8_t address, std::string data)
{
  uint8_t type = 0;
  if (data.length() > 0)
  {
    type |= PACKET_HAS_DATA;
  }
  if (data.length() > 4)
  {
    type |= PACKET_IS_BATCH;
    type |= (data.length() / 4) << PACKET_BATCH_LENGTH_OFFSET;
  }

  std::stringstream ss(std::stringstream::out | std::stringstream::binary);
  ss << "snp" << type << address << data;
  std::string output = ss.str();
  std::string c = checksum(output);
  ss << c;
  output = ss.str();
  ROS_DEBUG("Generated message %02x of overall length %zd.", address, output.length());

  // std::stringstream ss2;
  // for(char& c : output){
  //   if (c == 's' || c == 'n' ||c == 'p'){
  //     ss2 << c << ' ';
  //   } else{
  //     ss2 << std::hex << std::setfill('0') << std::setw(2) << int(c & 0xff) << ' ';
  //   }
    
  // }
  // ROS_WARN_STREAM("packet: " << ss2.str());
  return output;
}

void Comms::send(const Accessor_& r) const
{
  uint8_t address = r.index;
  std::string data(reinterpret_cast<char*>(r.raw()), r.length * 4);
  serial_->write(message(r.index, data));
}

void Comms::sendReadRegister(const Accessor_& r) const
{
  uint8_t address = r.index;
  serial_->write(message(r.index, std::string()));        // Send packet with empty string to read data
}

bool Comms::sendWaitAck(const Accessor_& r)
{
  const uint8_t tries = 5;
  for (uint8_t t = 0; t < tries; t++)
  {
    send(r);
    const uint8_t listens = 20;
    for (uint8_t i = 0; i < listens; i++)
    {
      int16_t received = receive();
      if (received == r.index)
      {
        ROS_DEBUG("Message %02x ack received.", received);
        return true;
      }
      else if (received == -1)
      {
        ROS_DEBUG("Serial read timed out waiting for ack. Attempting to retransmit.");
        break;
      }
    }
  }
  ROS_WARN("@@@ Failed to get ack!");
  return false;
}

bool Comms::sendWaitAck2(const Accessor_& r, std::string* data_str, uint8_t& packet_type, bool force_read_reg)
{
  const uint8_t tries = 3;
  for (uint8_t t = 0; t < tries; t++)
  {
    // flush serial before sending packet
    serial_->flushInput();
    if (force_read_reg)
    {
      sendReadRegister(r);
    }
    else
    {
      send(r);
    }
    // Wait for transmit to end
    serial_->flush();

    // ROS_DEBUG("^^ Packet SENT!");
    auto begin_time = std::chrono::steady_clock::now();
    
    try
    {
      if (receiveSpecificPacket(r, data_str, packet_type))
      {
        // auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count();
        // ROS_WARN("^^^ Message ack received. %ld ms", time_ms);
        return true;
      } 
      else
      {
        // auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time).count();
        // ROS_WARN("Serial read timed out waiting for ack. Attempting to retransmit. %ld", time_ms);
        continue;
      }
    }
    catch (const SerialTimeout& e)
    {
      // ROS_WARN("Serial timed-out! Attempting to re-send!");
      continue;
    }
  }
  ROS_ERROR("@@@ Failed to get ack!");
  return false;
}
}  // namespace um7
