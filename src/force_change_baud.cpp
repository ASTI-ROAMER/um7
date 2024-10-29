/**
 *
 *  \file
 *  \brief      Main entry point for UM7 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6)
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  \author     Alex Brown <rbirac@cox.net>		    (adapted to UM7)
 *  \copyright  Copyright (c) 2015, Alex Brown.
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
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. OR ALEX BROWN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <string>

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "um7/comms.h"
#include "um7/registers.h"
#include "um7/Reset.h"
#include <algorithm>

const char VERSION[10] = "0.0.2";   // um7_driver version

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = DREG_EULER_PHI_THETA;


// Just to simplify connecting to serial
bool connectToSerial(serial::Serial& ser, const int32_t& baud_rate){
  try
  {
    if (ser.isOpen())   ser.close();

    ser.setBaudrate(baud_rate);
    ROS_WARN("* Connecting to serial with baud rate:: %d ", baud_rate);
    ser.open();
    ser.flush();
    ser.flushInput();
    ser.flushOutput();
    ros::Duration(0.5).sleep();

    if (ser.isOpen()) {
      return true;
    }
    else {
      return false;
    }
    
  }
  catch (const serial::IOException& e)
  {
    ROS_WARN("* Unable to connect to serial port %s, baud: %d.", ser.getPort().c_str(), baud_rate);
    return false;
  }
}


/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um7_find_baud");

  // Load parameters from private node handle.
  std::string port;
  int32_t init_baud, final_baud, serial_baud;
  int receive_timeout_ms;
  std::set<int32_t> tried_baud_rates, valid_unix_baud_rates;
  std::set_intersection(um7::VALID_BAUD_RATES.begin(),
                        um7::VALID_BAUD_RATES.end(),
                        um7::UNIX_BAUD_RATES.begin(),
                        um7::UNIX_BAUD_RATES.end(),
                        std::inserter(valid_unix_baud_rates, valid_unix_baud_rates.end()));
  

  ros::NodeHandle imu_nh("imu"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("init_baud", init_baud, 115200);
  private_nh.param<int32_t>("final_baud", final_baud, 115200);
  private_nh.param<int>("receive_timeout_ms", receive_timeout_ms, 300);

  // Check if the baud rate is valid
  if(um7::VALID_BAUD_RATES.count(init_baud) && um7::UNIX_BAUD_RATES.count(init_baud) &&
      um7::VALID_BAUD_RATES.count(final_baud) && um7::UNIX_BAUD_RATES.count(final_baud))
  {
    ROS_DEBUG("Baud rates (init: %d, final:%d) are valid", init_baud, final_baud);
    serial_baud = init_baud;
  }
  else
  {
    std::string valid_bauds_str;
    for (auto it=valid_unix_baud_rates.begin(); it != valid_unix_baud_rates.end(); ++it){
      if (it != valid_unix_baud_rates.begin()){
        valid_bauds_str += ", ";
      }
      valid_bauds_str += std::to_string(*it);
    }

    ROS_ERROR("Init(%d) and/or final(%d) baud rates are INVALID. Valid rates: %s.", init_baud, final_baud, valid_bauds_str.c_str());
    ROS_ERROR("Exiting...");
    
    ros::shutdown();
    return -1;
  }

  // Check if timeout is valid
  if(receive_timeout_ms >= 0)
  {
    ROS_DEBUG("Value for receive_timeout_ms: %d\n", receive_timeout_ms);
  }
  else
  {
    ROS_ERROR("Value for receive_timeout_ms should be an integer >= 0\n");
  }

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(init_baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);


  // STEPS:
  //  1) Connect with init_baud
  //  2) Verify connection with firmware version
  //  3) Update baud rate of device (RAM only) to final_baud. We don't care about the ack for this.
  //  4) Reconnect to device with final_baud. We don't care to verify here.
  //  5) Flash the settings.
  enum BaudChangeStepsEnum{
    DISCONNECTED = 0,
    CONNECTED_INIT_BAUD,
    VERIFIED_INIT_BAUD,
    UPDATED_UM7_BAUD_TO_FINAL_BAUD,
    CONNECTED_FINAL_BAUD,
    FLASHED_FINAL_BAUD
  } state = DISCONNECTED;



  um7::Comms sensor(&ser, (unsigned int)receive_timeout_ms);
  um7::Registers r;
  std::string fw_str;
  //  1) Connect with init_baud
  if (!connectToSerial(ser, init_baud)) {
    ROS_WARN("* Unable to connect to serial port %s, baud: %d.", port.c_str(), init_baud);
    ros::shutdown();
    return -1;
  }


  //  2) Verify connection with firmware version, for init_baud
  ROS_WARN("** VALIDATING connection to UM7 by querying its firmware version...");
  if (sensor.sendWaitAck2(r.cmd_get_fw_revision, &fw_str, true)) {
    ROS_WARN("*** UM7 Firmware version:%s", fw_str.c_str());
    ROS_WARN("*** We are connected to UM7 with init_baud(%d).", (int)init_baud);
    ros::Duration(0.5).sleep();
  } 
  else {
    ROS_ERROR("Validation FAILED for init_baud(%d). Now exiting...", ser.getBaudrate());
    ser.close();
    ros::shutdown();
    return -1;
  }
  
  //  3) Update baud rate of device (RAM only) to final_baud. We don't care about the ack for this.
  ROS_WARN("** Now setting the device to the final_baud(%d), RAM only, and will reconnect after!", final_baud);
  // Find index of the user baud in um7::VALID_BAUD_RATES
  auto it_= um7::VALID_BAUD_RATES.find(final_baud);
  uint32_t final_baud_val = (uint32_t)std::distance(um7::VALID_BAUD_RATES.begin(), it_);
  uint32_t comm_reg = (final_baud_val << COM_BAUD_START);    // uhh, this will actually change the whole register without preserving the other previous setting
  // ROS_WARN("*** Setting CREG_COM_SETTINGS to: 0x%02X", comm_reg);
  r.communication.set(0, comm_reg);
  // We don't need to wait for ack when we change the baud rate, since the reply will be in a different baudrate, thus unreadable.
  sensor.send(r.communication);
  ros::Duration(1.0).sleep();


  //  4) Reconnect to device with final_baud. We don't care to verify here.
  if (!connectToSerial(ser, final_baud)) {
    ROS_WARN("* Unable to connect to serial port %s, baud: %d.", port.c_str(), final_baud);
    ROS_WARN("* [IMPORTANT] We already changed the baud rate settings of UM7 to %d (not yet flashed!).", final_baud);
    ros::shutdown();
    return -1;
  }

  //  5) Flash the settings.
  ros::Duration(0.5).sleep();
  ROS_WARN("** Now FLASHING the final_baud(%d)!", final_baud);
  if (sensor.sendWaitAck2(r.cmd_flash_commit, true))
  {
    ROS_WARN("*** Flashing SUCCESS! Baud rate is %d", final_baud);
  }
  else {
    ROS_WARN("*** Flashing was not acknowledged. Disconnect then reconnect the device to verify that it was flashed with baud rate of %d.", final_baud);
  }

  // exit 
  if (ser.isOpen())  ser.close();
  ros::shutdown();
  return 0;
}
