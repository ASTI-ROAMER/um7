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


/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um7_find_baud");

  // Load parameters from private node handle.
  std::string port;
  int32_t user_baud, serial_baud;
  int receive_timeout_ms;
  std::set<int32_t> tried_baud_rates, valid_unix_baud_rates;
  std::set_intersection(um7::VALID_BAUD_RATES.begin(),
                        um7::VALID_BAUD_RATES.end(),
                        um7::UNIX_BAUD_RATES.begin(),
                        um7::UNIX_BAUD_RATES.end(),
                        std::inserter(valid_unix_baud_rates, valid_unix_baud_rates.end()));
  

  ros::NodeHandle imu_nh("imu"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("baud", user_baud, 115200);
  private_nh.param<int>("receive_timeout_ms", receive_timeout_ms, 300);

  // Check if the baud rate is valid
  if(um7::VALID_BAUD_RATES.count(user_baud) && um7::UNIX_BAUD_RATES.count(user_baud))
  {
    ROS_DEBUG("Baud rate (%d) is valid", user_baud);
    serial_baud = user_baud;
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

    if(um7::VALID_BAUD_RATES.count(user_baud)){
      ROS_ERROR("Baud rate (%d) is SUPPORTED BY UM7, but is NOT SUPPORTED by UNIX. Valid rates: %s. Will use the default baud rate(115200) to start searching.", user_baud, valid_bauds_str.c_str());
    } 
    else
    {
      ROS_ERROR("Baud rate (%d) is INVALID. Valid rates: %s. Will use the default baud rate(115200) to start searching.", user_baud, valid_bauds_str.c_str());
    }
    
    user_baud = 115200;
    serial_baud = 115200;
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
  ser.setBaudrate(serial_baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);


  // Real Time Loop
  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.setBaudrate(serial_baud);
      ROS_WARN("\n\n====================================");
      ROS_WARN("===== TRYING BAUD serial_baud: %d ", serial_baud);
      ros::Duration(1.0).sleep();
      ser.open();
      ser.flush();
      ser.flushInput();
      ser.flushOutput();
      tried_baud_rates.insert(ser.getBaudrate());
    }
    catch (const serial::IOException& e)
    {
        ROS_WARN("* Unable to connect to serial port %s, baud: %d.", port.c_str(), serial_baud);
    }

    if (ser.isOpen())
    {
      ROS_WARN("* Successfully connected to serial port %s, baud: %d.", port.c_str(), serial_baud);
      first_failure = true;
      try
      {
        ROS_WARN("** VALIDATING connection to UM7 by querying its firmware version...");
        um7::Comms sensor(&ser, (unsigned int)receive_timeout_ms);
        um7::Registers r;
        std::string fw_str;
        if (!sensor.sendWaitAck2(r.cmd_get_fw_revision, &fw_str, true)) {
          throw um7::DeviceWrongBaud("!!! Validation FAILED! Serial baud (" + std::to_string(serial_baud) +") probably is different from the device's baud setting.");
        } 
        else {
          ROS_WARN("*** UM7 Firmware version:%s", fw_str.c_str());
          ROS_WARN("*** Found UM7's baud rate: %d. Now exiting!", (int)serial_baud);
          ser.close();
          ros::shutdown();
          return 0;
        }
      }
      catch(const um7::DeviceWrongBaud& e)
      {
        // std::string stty_stdout;
        // int stty_exit_status;
        // stty_stdout = execCommand("stty -F /dev/ttyUSB0 speed", stty_exit_status);
        // ROS_WARN("#### \nSTTY end SPEED:%s\n", stty_stdout.c_str());

        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());

        std::set<int32_t> not_tried_baud;
        std::set_difference(valid_unix_baud_rates.begin(), 
                            valid_unix_baud_rates.end(), 
                            tried_baud_rates.begin(), 
                            tried_baud_rates.end(),
                            std::inserter(not_tried_baud, not_tried_baud.end()));
        int32_t try_next_baud;
        if (tried_baud_rates.count(um7::DEFAULT_BAUD_RATE) == 0){
          ROS_ERROR("!!! Will try to connect next time with default baud rate: %d", um7::DEFAULT_BAUD_RATE);
          serial_baud = um7::DEFAULT_BAUD_RATE;
        }
        else if (not_tried_baud.size() == 0)
        {
          ROS_ERROR("!!! Can't find valid baud rate to connect to UM7! Now exiting...");
          ros::shutdown();
          return -1;

          // ROS_ERROR("!!! Re-cycling connection with valid baud rates...");
          // tried_baud_rates.clear();
          // // go back to user-set baud
          // serial_baud = user_baud;
        }
        else
        {
          serial_baud = *(not_tried_baud).rbegin();
          ROS_WARN("!!! Will try to connect next time with baud rate: %d", serial_baud);
        }

        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
                           << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
