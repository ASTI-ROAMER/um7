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
  int32_t baud_rate, serial_baud;
  int receive_timeout_ms;
  std::set<int32_t> tried_baud_rates, valid_unix_baud_rates;
  std::set_intersection(um7::VALID_BAUD_RATES.begin(),
                        um7::VALID_BAUD_RATES.end(),
                        um7::UNIX_BAUD_RATES.begin(),
                        um7::UNIX_BAUD_RATES.end(),
                        std::inserter(valid_unix_baud_rates, valid_unix_baud_rates.end()));
  

  ros::NodeHandle imu_nh("imu"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("baud", baud_rate, 115200);
  private_nh.param<int>("receive_timeout_ms", receive_timeout_ms, 300);

  // Check if the baud rate is valid
  if(um7::VALID_BAUD_RATES.count(baud_rate) && um7::UNIX_BAUD_RATES.count(baud_rate) )
  {
    ROS_DEBUG("Baud rate (%d) is valid", baud_rate);
    serial_baud = baud_rate;
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

    ROS_ERROR("Baud rate (%d) is INVALID. Valid rates: %s.", baud_rate, valid_bauds_str.c_str());
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
  ser.setBaudrate(baud_rate);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);






  um7::Comms sensor(&ser, (unsigned int)receive_timeout_ms);
  um7::Registers r;
  std::string fw_str;
  //  1) Connect with init_baud
  if (!connectToSerial(ser, baud_rate)) {
    ROS_WARN("* Unable to connect to serial port %s, baud: %d.", port.c_str(), baud_rate);
    ros::shutdown();
    return -1;
  }


  //  5) Flash the settings.
  ros::Duration(0.5).sleep();
  ROS_WARN("** Now FLASHING, baud: %d!", baud_rate);
  if (sensor.sendWaitAck2(r.cmd_flash_commit, true))
  {
    ROS_WARN("*** Flashing SUCCESS! Baud rate is %d", baud_rate);
  }
  else {
    ROS_WARN("*** Flashing was not acknowledged. Disconnect then reconnect the device to verify that it was flashed with baud rate of %d.", baud_rate);
  }

  // exit 
  if (ser.isOpen())  ser.close();
  ros::shutdown();
  return 0;
}
