/**
 *
 *  \file
 *  \brief      Comms class definition. Does not manage the serial connection
 *              itself, but takes care of reading and writing to UM7.
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

#ifndef UM7_COMMS_H_
#define UM7_COMMS_H_

#include <stdint.h>
#include <string>
#include <set>
#include <stdexcept>

namespace serial
{
class Serial;
}

namespace um7
{

class SerialTimeout : public std::exception {};

class BadChecksum : public std::exception {};

struct DeviceWrongBaud : public std::runtime_error { using std::runtime_error::runtime_error; };

struct DeviceBaudChanged : public std::runtime_error { using std::runtime_error::runtime_error; };

struct WriteConfigNoAck : public std::runtime_error { using std::runtime_error::runtime_error; };

class Registers;
class Accessor_;

const std::set<int32_t> VALID_BAUD_RATES = {9600, 
                                            14400,
                                            19200,
                                            38400,
                                            57600,
                                            115200,
                                            128000,
                                            153600,
                                            230400,
                                            256000,
                                            460800,
                                            921600};
                                            
const std::set<int32_t> UNIX_BAUD_RATES = { 9600, 
                                            19200,
                                            38400,
                                            57600,
                                            115200,
                                            153600,
                                            230400,
                                            460800,
                                            921600};

const int32_t DEFAULT_BAUD_RATE = 115200;

class Comms
{
public:
  explicit Comms(serial::Serial* s) : serial_(s), first_spin_(true), wait_ack_timeout_ms(300)
  {
  }

  Comms(serial::Serial* s, unsigned int _wait_ack_timeout_ms) : serial_(s), first_spin_(true), wait_ack_timeout_ms(_wait_ack_timeout_ms)
  {
  }

  /**
   * Returns -1 if the serial port timed out before receiving a packet
   * successfully, or if there was a bad checksum or any other error.
   * Otherwise, returns the 8-bit register number of the successfully
   * returned packet.
   */
  int16_t receive(Registers* r);
  // size_t Comms::readline_recycle (std::string &buffer, size_t size, std::string eol);
  // This will block and read serial until timeout. Will not flush buffer.

  // New packet detection, that re-implements readline to be inside a timeout. 
  // This will block and read serial until timeout. Will not flush buffer.
  bool receiveSpecificPacket(const Accessor_& a) {
    return receiveSpecificPacket(a, nullptr);
  };
  bool receiveSpecificPacket(const Accessor_& a, std::string* data_sptr) {
    return receiveSpecificPacket(a, data_sptr, wait_ack_timeout_ms);
  };
  bool receiveSpecificPacket(const Accessor_& a, std::string* data_sptr, const unsigned int& _wait_ack_timeout_ms)
  {
    uint8_t packet_type;      // this effectively discards the packet type reply
    return receiveSpecificPacket(a, data_sptr, packet_type, _wait_ack_timeout_ms);
  };
  bool receiveSpecificPacket(const Accessor_& a, std::string* data_sptr, uint8_t& packet_type)
  {
    return receiveSpecificPacket(a, data_sptr, packet_type, wait_ack_timeout_ms);
  };
  bool receiveSpecificPacket(const Accessor_& a, std::string* data_sptr, uint8_t& packet_type, const unsigned int& _wait_ack_timeout_ms);

  void send(const Accessor_& a) const;

  // We need this cuz using send() will additionally write the configured data length of the accessor. 
  // This means send() will ALWAYS WRITE data if you give it an accessor with a register that has length > 0.
  void sendReadRegister(const Accessor_& r) const;

  bool sendWaitAck(const Accessor_& a);


  bool sendWaitAck2(const Accessor_& a)
  {
    return sendWaitAck2(a, nullptr, false);
  };  
  bool sendWaitAck2(const Accessor_& a, std::string* data_sptr)
  {
    return sendWaitAck2(a, data_sptr, false);
  };  
  bool sendWaitAck2(const Accessor_& a, bool force_read_reg)
  {
    return sendWaitAck2(a, nullptr, force_read_reg);
  };  
  bool sendWaitAck2(const Accessor_& a, std::string* data_str, bool force_read_reg)
  {
    uint8_t packet_type;      // this effectively discards the packet type reply
    return sendWaitAck2(a, data_str, packet_type, force_read_reg);
  };
  bool sendWaitAck2(const Accessor_& a, std::string* data_str, uint8_t& packet_type)
  {
    return sendWaitAck2(a, data_str, packet_type, false);
  };
  bool sendWaitAck2(const Accessor_& a, std::string* data_str, uint8_t& packet_type, bool force_read_reg);

    


  static const uint8_t PACKET_HAS_DATA;
  static const uint8_t PACKET_IS_BATCH;
  static const uint8_t PACKET_BATCH_LENGTH_MASK;
  static const uint8_t PACKET_BATCH_LENGTH_OFFSET;

  static std::string checksum(const std::string& s);

  static std::string message(uint8_t address, std::string data);

private:
  bool first_spin_;
public:
  serial::Serial* serial_;
  unsigned int wait_ack_timeout_ms;
};
}  // namespace um7

#endif  // UM7_COMMS_H

