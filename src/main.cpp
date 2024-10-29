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

namespace OutputAxisOptions
{
  enum OutputAxisOption
  {
    DEFAULT, ENU, ROBOT_FRAME
  };
}
typedef OutputAxisOptions::OutputAxisOption OutputAxisOption;

/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in UM7 registers.
 */
template<typename RegT>
void configureVector3(um7::Comms* sensor, const um7::Accessor<RegT>& reg,
                      std::string param, std::string human_name)
{
  if (reg.length != 3)
  {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (sensor->sendWaitAck2(reg))
    {
      throw std::runtime_error("Unable to configure vector.");
    }
  }
}

/**
 * Function generalizes the process of commanding the UM7 via one of its command
 * registers.
 */
template<typename RegT>
bool sendCommand(um7::Comms* sensor, const um7::Accessor<RegT>& reg, std::string human_name)
{
  ROS_WARN("*** Sending command: %s", human_name.c_str());
  std::string data_str;
  uint8_t packet_type;
  if (!sensor->sendWaitAck2(reg, &data_str, packet_type, true)){
    // throw std::runtime_error("Command to device failed.");
    return false;
  }
  else {
    // Check if the Command Failed bit of the Packet_Type byte is set 
    bool cmd_failed = bool(packet_type & 0x01);
    if (cmd_failed){
      ROS_WARN("---- Command [%s](0x%02X) FAILED!", human_name.c_str(), reg.index);
    } 
    else{
      ROS_WARN("---- Command [%s](0x%02X) SUCCEEDED!", human_name.c_str(), reg.index);
    }

    // If the command has a reply
    if (!data_str.empty()){
      ROS_WARN("*** Got a reply for %s: %s", human_name.c_str(), data_str.c_str());
    }
    return true;
  }
  
}

// This can be also used to check if we can communicate to the UM7
bool verifyFirmwareVersion(um7::Comms* sensor, std::string& fw_str){
  fw_str.clear();
  if (sensor->serial_->isOpen()) {
    um7::Registers r;
    // Verify if the baud rate use to connect to the serial port is the same as the device's configured baud rate by querying the device firmware version
    ROS_WARN("*** VALIDATING connection to UM7...");
    if (sensor->sendWaitAck2(r.cmd_get_fw_revision, &fw_str, true))
    {
      ROS_WARN("---- UM7 Firmware version: %s", fw_str.c_str());
      return true;
    }
    else {
      ROS_ERROR("---- Validation FAILED! Serial baud (%s) probably is different from the device's baud setting.", std::to_string(sensor->serial_->getBaudrate()).c_str());
      return false;
    } 
  }
  return false;
}

// Check if UM7 is Overflowing. 
// Overflow is when the UM7 is attempting to transmit data over the serial port faster than is allowed given the baud-rate.
// -1 = error, 0 = no overflow(ok), 1 = overflow(not ok)
int checkOverflow(um7::Comms* sensor){
  if (sensor->serial_->isOpen()) {
    um7::Registers r;
    ROS_WARN("*** Reading DREG_HEALTH to check for overflow...");
    std::string health_str;
    try {
      if( sensor->sendWaitAck2(r.health, &health_str, true) ) {
        uint32_t health_reg = (uint32_t)health_str[0] << 24 |
                              (uint32_t)health_str[1] << 16 |
                              (uint32_t)health_str[2] << 8  |
                              (uint32_t)health_str[3];
        if (bool(health_reg & HEALTH_COM_OVERFLOW)){
          ROS_ERROR("---- UM7 reports OVEFLOW! Reduce [update_rate] parameter or increase [baud] rate (change baud rate first via force_change_baud).");
          return 1;
        }
        else {
          ROS_WARN("---- Overflow not detected!");
          return 0;
        }
      }
      else {
        ROS_ERROR("---- Unable to read DREG_HEALTH for OVERFLOW.");
        return -1;
      }
    }
    catch(const um7::SerialTimeout& e) {
      ROS_ERROR("SerialTimeout: Reading parts of the health packet reply timed-out.");
    }
    catch(const um7::BadChecksum& e) {
      ROS_ERROR("BadChecksum: Found the health packet, but with a bad checksum.");
      throw;
    }
    
  }
  return -1;
}


/**
 * Send configuration messages to the UM7, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(um7::Comms* sensor, ros::NodeHandle *private_nh, int32_t& user_baud, int32_t& serial_baud)
{
  um7::Registers r;
  bool ignore_unacknowledged_configs;
  private_nh->param<bool>("ignore_unacknowledged_configs", ignore_unacknowledged_configs, true);

  
  // DO NOT SET THE BAUD RATE OF THE DEVICE!!!!!
  std::string fw_str;
  bool verify_fw_before_config;
  private_nh->param<bool>("verify_fw_before_config", verify_fw_before_config, false);
  if ( !verifyFirmwareVersion(sensor, fw_str) ) {
    if (verify_fw_before_config) {
      throw um7::DeviceWrongBaud("Aborting configuration since [verify_fw_before_config] was set to True. Exiting...");
    }
    else {
      ROS_ERROR("---- IGNORING firmware validation failure. Continuing connection...");
    }
  }


  // // Read the device's configured baud rate. At this point, the serial baud and device baud matches already.
  // ROS_WARN("\n\nTrying to query UM7 Baud rate settings");
  // std::string comm_settings_str;
  // if (!sensor->sendWaitAck2(r.communication, &comm_settings_str, true))
  // {
  //   throw std::runtime_error("!!! Unable to read CREG_COM_SETTINGS for BAUD_RATE.");
  // } 

  // int32_t dev_baud;
  // if (comm_settings_str.length() == 4)
  // {
  //   uint32_t comm_settings_uint32 = (uint32_t)comm_settings_str[0] << 24 |
  //                                   (uint32_t)comm_settings_str[1] << 16 |
  //                                   (uint32_t)comm_settings_str[2] << 8 |
  //                                   (uint32_t)comm_settings_str[3];
  //   // Extract from baud setting from comm settings register.
  //   uint8_t dev_baud_uint8 = (uint8_t)((comm_settings_uint32 >> COM_BAUD_START) & COM_BAUD_MASK);
    
  //   // Validate that the just read baud setting has correct value (from datasheet).
  //   if (dev_baud_uint8 >= um7::VALID_BAUD_RATES.size()) throw std::runtime_error("The configured value for BAUD_RATE is probably wrong.");
    
  //   // Access VALID_BAUD_RATES set like an array, to get to corresponding baud rate value 
  //   auto it_dev_baud = std::next(um7::VALID_BAUD_RATES.begin(), dev_baud_uint8);
  //   dev_baud = *it_dev_baud;
  //   // ROS_WARN("**** a:%d, s:%d, u:%d, d:%d", (int)(sensor->serial_->getBaudrate()), (int)serial_baud, (int)user_baud, (int)dev_baud);
  // }
  // else
  // {
  //   throw std::runtime_error("Wrong data reply size for CREG_COM_SETTINGS for BAUD_RATE.");
  // }
  

  // // Check if the device's configured baud rate is the same as what user has given. 
  // if(dev_baud == user_baud)
  // {
  //   ROS_WARN("*** The user specified baud rate (%d) MATCHES the de ice's baud rate (%d)", user_baud, dev_baud);
  // }
  // else      // If dev_baud and user_baud DON'T match, correct the device's baud rate.
  // {
  //   ROS_WARN("*** The serial(%d) and device(%d) baud rates DON'T MATCH with the user baud (%d)!", serial_baud, dev_baud, user_baud);
  //   ROS_WARN("*** Now setting the device to the user baud(%d), and will reconnect after!", user_baud);
  //   // Find index of the user baud in um7::VALID_BAUD_RATES
  //   auto it_= um7::VALID_BAUD_RATES.find(user_baud);
  //   uint32_t user_baud_val = (uint32_t)std::distance(um7::VALID_BAUD_RATES.begin(), it_);
  //   // ROS_WARN("**** user_baud(%d) user_baud_val: %d",user_baud, int(user_baud_val));

  //   uint32_t comm_reg = (user_baud_val << COM_BAUD_START);    // uhh, this will actually change the whole register without preserving the other previous setting
  //   // ROS_WARN("*** Setting CREG_COM_SETTINGS to: 0x%02X", comm_reg);
  //   r.communication.set(0, comm_reg);

  //   // We don't need to wait for ack when we change the baud rate, since the reply will be in a different baudrate, thus unreadable.
  //   sensor->send(r.communication);
  //   throw um7::DeviceBaudChanged("We just updated UM7 Baud setting to user_baud (" +std::to_string(user_baud) + ").");     // This will be caught by main, and will reconnect to serial using user_baud
  // }
  
  ROS_WARN("** Updating configuration registers...");
  // set the broadcast rate of the device
  int rate;
  private_nh->param<int>("update_rate", rate, 20);
  if (rate < 20 || rate > 255)
  {
    ROS_WARN("Potentially unsupported update rate of %d", rate);
  }
  uint32_t rate_bits = static_cast<uint32_t>(rate);
  

  ROS_WARN("*** Trying to change CREG_COM_RATES1");
  uint32_t raw_rate_separate = 0;
  ROS_WARN("**** Setting update rate (separate) to 0 Hz (0x%08X)", raw_rate_separate);
  r.comrate1.set(0, raw_rate_separate);
  if (!sensor->sendWaitAck2(r.comrate1))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES1.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }

  }
  else {
    ROS_WARN("---- OK CREG_COM_RATES1");
  }


  ROS_WARN("*** Trying to change CREG_COM_RATES2");
  uint32_t raw_rate = 0;        //(rate_bits << RATE2_ALL_RAW_START);
  ROS_WARN("**** Setting update rate to 0 Hz (0x%08X)", raw_rate);
  r.comrate2.set(0, raw_rate);
  if (!sensor->sendWaitAck2(r.comrate2))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES2.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }

  }
  else {
    ROS_WARN("---- OK CREG_COM_RATES2");
  }


  // REDUNDANT since we will be setting CREG_COM_RATES4 properly
  // ROS_WARN("*** Trying to change CREG_COM_RATES3");
  // uint32_t proc_rate_separate = 0;
  // ROS_WARN("**** Setting update rate (separate) to 0Hz (0x%08X)", proc_rate_separate);
  // r.comrate3.set(0, proc_rate_separate);
  // if (!sensor->sendWaitAck2(r.comrate3))
  // {
  //   if (!ignore_unacknowledged_configs) {
  //     throw std::runtime_error("Unable to set CREG_COM_RATES3.");
  //   }
  //   else {
  //     ROS_ERROR("---- Ignoring unacknowledged configuration!");
  //   }

  // }
  // else {
  //   ROS_WARN("---- OK CREG_COM_RATES3");
  // }
  

  ROS_WARN("*** Trying to change CREG_COM_RATES4");
  uint32_t proc_rate = (rate_bits << RATE4_ALL_PROC_START);
  ROS_WARN("**** Setting update rate to %d Hz (0x%08X)", rate, proc_rate);
  r.comrate4.set(0, proc_rate);
  if (!sensor->sendWaitAck2(r.comrate4))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES4.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }
  } 
  else {
    ROS_WARN("---- OK CREG_COM_RATES4");
  }
  

  ROS_WARN("*** Trying to change CREG_COM_RATES5");
  uint32_t misc_rate = (rate_bits << RATE5_EULER_START) | (rate_bits << RATE5_QUAT_START);
  ROS_WARN("**** Setting update rate to %d Hz (0x%08X)", rate, misc_rate);
  r.comrate5.set(0, misc_rate);
  if (!sensor->sendWaitAck2(r.comrate5))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES5.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }
  }
  else {
    ROS_WARN("---- OK CREG_COM_RATES5");
  }
  

  ROS_WARN("*** Trying to change CREG_COM_RATES6");
  uint32_t health_rate = (5 << RATE6_HEALTH_START);  // note:  5 gives 2 hz rate
  ROS_WARN("**** Setting update rate to 5 Hz (0x%08X)", health_rate);
  r.comrate6.set(0, health_rate);
  if (!sensor->sendWaitAck2(r.comrate6))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES6.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }
  }
  else {
    ROS_WARN("---- OK CREG_COM_RATES6");
  }
  

  ROS_WARN("*** Trying to change CREG_COM_RATES7");
  uint32_t nmea_rate = 0;
  ROS_WARN("**** Setting update rate to 0 Hz (0x%08X)", nmea_rate);
  r.comrate7.set(0, nmea_rate);
  if (!sensor->sendWaitAck2(r.comrate7))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_COM_RATES7.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }
  }
  else {
    ROS_WARN("---- OK CREG_COM_RATES7");
  }

  ROS_WARN("*** Trying to change CREG_MISC_SETTINGS");
  // Options available using parameters)
  uint32_t misc_config_reg = 0;  // initialize all options off
  // Optionally disable mag updates in the sensor's EKF.
  bool mag_updates;
  private_nh->param<bool>("mag_updates", mag_updates, true);
  if (mag_updates)
  {
    misc_config_reg |= MAG_UPDATES_ENABLED;
    ROS_WARN("**** Including magnetometer updates from EKF.");
  }
  else
  {
    ROS_WARN("**** Excluding magnetometer updates from EKF.");
  }

  // Optionally enable quaternion mode .
  bool quat_mode;
  private_nh->param<bool>("quat_mode", quat_mode, true);
  if (quat_mode)
  {
    misc_config_reg |= QUATERNION_MODE_ENABLED;
    ROS_WARN("**** Including quaternion mode.");
  }
  else
  {
    ROS_WARN("**** Excluding quaternion mode.");
  }

  r.misc_config.set(0, misc_config_reg);
  if (!sensor->sendWaitAck2(r.misc_config))
  {
    if (!ignore_unacknowledged_configs) {
      throw std::runtime_error("Unable to set CREG_MISC_SETTINGS.");
    }
    else {
      ROS_ERROR("---- Ignoring unacknowledged configuration!");
    }
  }
  else {
    ROS_WARN("---- OK CREG_MISC_SETTINGS");
  }
  

  // Optionally disable performing a zero gyros command on driver startup.
  bool zero_gyros;
  private_nh->param<bool>("zero_gyros", zero_gyros, true);
  if (zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");

  checkOverflow(sensor);

  bool flash_config;
  private_nh->param<bool>("flash_config", flash_config, false);
  if(flash_config)
  {
    ROS_WARN("Flashing the settings...");
    if (!sensor->sendWaitAck2(r.cmd_flash_commit, true))
    {
      if (!ignore_unacknowledged_configs) {
        throw std::runtime_error("Unable to FLASH_COMMIT!");
      }
      else {
        ROS_ERROR("---- Ignoring unacknowledged configuration!");
      }
    }
    else {
      ROS_WARN("---- Flashing SUCCESS!!!");
    }
  }


  ROS_WARN("******* UM7 Configuration done!!! *******");
  ROS_WARN("*****************************************\n");
}


bool handleResetService(um7::Comms* sensor,
    const um7::Reset::Request& req, const um7::Reset::Response& resp)
{
  um7::Registers r;
  if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
  if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um7::Registers& r, ros::NodeHandle* imu_nh, sensor_msgs::Imu& imu_msg,
    OutputAxisOption axes, bool use_magnetic_field_msg)
{
  static ros::Publisher imu_pub = imu_nh->advertise<sensor_msgs::Imu>("data", 1, false);
  static ros::Publisher mag_pub;
  if (use_magnetic_field_msg)
  {
    mag_pub = imu_nh->advertise<sensor_msgs::MagneticField>("mag", 1, false);
  }
  else
  {
    mag_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("mag", 1, false);
  }
  static ros::Publisher rpy_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("rpy", 1, false);
  static ros::Publisher temp_pub = imu_nh->advertise<std_msgs::Float32>("temperature", 1, false);

  if (imu_pub.getNumSubscribers() > 0)
  {
    switch (axes)
    {
      case OutputAxisOptions::ENU:
      {
        // body-fixed frame NED to ENU: (x y z)->(x -y -z) or (w x y z)->(x -y -z w)
        // world frame      NED to ENU: (x y z)->(y  x -z) or (w x y z)->(y  x -z w)
        // world frame
        imu_msg.orientation.w =  r.quat.get_scaled(2);
        imu_msg.orientation.x =  r.quat.get_scaled(1);
        imu_msg.orientation.y = -r.quat.get_scaled(3);
        imu_msg.orientation.z =  r.quat.get_scaled(0);

        // body-fixed frame
        imu_msg.angular_velocity.x =  r.gyro.get_scaled(0);
        imu_msg.angular_velocity.y = -r.gyro.get_scaled(1);
        imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

        // body-fixed frame
        imu_msg.linear_acceleration.x =  r.accel.get_scaled(0);
        imu_msg.linear_acceleration.y = -r.accel.get_scaled(1);
        imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);
        break;
      }
      case OutputAxisOptions::ROBOT_FRAME:
      {
        // body-fixed frame
        imu_msg.orientation.w = -r.quat.get_scaled(0);
        imu_msg.orientation.x = -r.quat.get_scaled(1);
        imu_msg.orientation.y =  r.quat.get_scaled(2);
        imu_msg.orientation.z =  r.quat.get_scaled(3);

        // body-fixed frame
        imu_msg.angular_velocity.x =  r.gyro.get_scaled(0);
        imu_msg.angular_velocity.y = -r.gyro.get_scaled(1);
        imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

        // body-fixed frame
        imu_msg.linear_acceleration.x =  r.accel.get_scaled(0);
        imu_msg.linear_acceleration.y = -r.accel.get_scaled(1);
        imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);
        break;
      }
      case OutputAxisOptions::DEFAULT:
      {
        imu_msg.orientation.w = r.quat.get_scaled(0);
        imu_msg.orientation.x = r.quat.get_scaled(1);
        imu_msg.orientation.y = r.quat.get_scaled(2);
        imu_msg.orientation.z = r.quat.get_scaled(3);

        imu_msg.angular_velocity.x = r.gyro.get_scaled(0);
        imu_msg.angular_velocity.y = r.gyro.get_scaled(1);
        imu_msg.angular_velocity.z = r.gyro.get_scaled(2);

        imu_msg.linear_acceleration.x = r.accel.get_scaled(0);
        imu_msg.linear_acceleration.y = r.accel.get_scaled(1);
        imu_msg.linear_acceleration.z = r.accel.get_scaled(2);
        break;
      }
      default:
        ROS_ERROR("OuputAxes enum value invalid");
    }

    imu_pub.publish(imu_msg);
  }

  // Magnetometer.  transform to ROS axes
  if (mag_pub.getNumSubscribers() > 0)
  {
    if (use_magnetic_field_msg)
    {
      sensor_msgs::MagneticField mag_msg;
      mag_msg.header = imu_msg.header;

      switch (axes)
      {
        case OutputAxisOptions::ENU:
        {
          mag_msg.magnetic_field.x = r.mag.get_scaled(1);
          mag_msg.magnetic_field.y = r.mag.get_scaled(0);
          mag_msg.magnetic_field.z = -r.mag.get_scaled(2);
          break;
        }
        case OutputAxisOptions::ROBOT_FRAME:
        {
          // body-fixed frame
          mag_msg.magnetic_field.x =  r.mag.get_scaled(0);
          mag_msg.magnetic_field.y = -r.mag.get_scaled(1);
          mag_msg.magnetic_field.z = -r.mag.get_scaled(2);
          break;
        }
        case OutputAxisOptions::DEFAULT:
        {
          mag_msg.magnetic_field.x = r.mag.get_scaled(0);
          mag_msg.magnetic_field.y = r.mag.get_scaled(1);
          mag_msg.magnetic_field.z = r.mag.get_scaled(2);
          break;
        }
        default:
          ROS_ERROR("OuputAxes enum value invalid");
      }

      mag_pub.publish(mag_msg);
    }
    else
    {
      geometry_msgs::Vector3Stamped mag_msg;
      mag_msg.header = imu_msg.header;

      switch (axes)
      {
        case OutputAxisOptions::ENU:
        {
          mag_msg.vector.x = r.mag.get_scaled(1);
          mag_msg.vector.y = r.mag.get_scaled(0);
          mag_msg.vector.z = -r.mag.get_scaled(2);
          break;
        }
        case OutputAxisOptions::ROBOT_FRAME:
        {
          // body-fixed frame
          mag_msg.vector.x =  r.mag.get_scaled(0);
          mag_msg.vector.y = -r.mag.get_scaled(1);
          mag_msg.vector.z = -r.mag.get_scaled(2);
          break;
        }
        case OutputAxisOptions::DEFAULT:
        {
          mag_msg.vector.x = r.mag.get_scaled(0);
          mag_msg.vector.y = r.mag.get_scaled(1);
          mag_msg.vector.z = r.mag.get_scaled(2);
          break;
        }
        default:
          ROS_ERROR("OuputAxes enum value invalid");
      }

      mag_pub.publish(mag_msg);
    }
  }

  // Euler attitudes.  transform to ROS axes
  if (rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = imu_msg.header;

    switch (axes)
    {
      case OutputAxisOptions::ENU:
      {
        // world frame
        rpy_msg.vector.x = r.euler.get_scaled(1);
        rpy_msg.vector.y = r.euler.get_scaled(0);
        rpy_msg.vector.z = -r.euler.get_scaled(2);
        break;
      }
      case OutputAxisOptions::ROBOT_FRAME:
      {
        rpy_msg.vector.x =  r.euler.get_scaled(0);
        rpy_msg.vector.y = -r.euler.get_scaled(1);
        rpy_msg.vector.z = -r.euler.get_scaled(2);
        break;
      }
      case OutputAxisOptions::DEFAULT:
      {
        rpy_msg.vector.x = r.euler.get_scaled(0);
        rpy_msg.vector.y = r.euler.get_scaled(1);
        rpy_msg.vector.z = r.euler.get_scaled(2);
        break;
      }
      default:
        ROS_ERROR("OuputAxes enum value invalid");
    }

    rpy_pub.publish(rpy_msg);
  }

  // Temperature
  if (temp_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
}

/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um7_driver");

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
      ROS_ERROR("Baud rate (%d) is SUPPORTED BY UM7, but is NOT SUPPORTED by UNIX. Valid rates: %s. Now exiting...", user_baud, valid_bauds_str.c_str());
    } 
    else
    {
      ROS_ERROR("Baud rate (%d) is INVALID. Valid rates: %s. Now exiting...", user_baud, valid_bauds_str.c_str());
    }
    
    ros::shutdown();
    return -1;
  }

  // Check if timeout is valid
  if(receive_timeout_ms >= 0)
  {
    ROS_WARN("Value for receive_timeout_ms: %d", receive_timeout_ms);
  }
  else
  {
    ROS_ERROR("Value for receive_timeout_ms should be an integer >= 0");
  }

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(serial_baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  sensor_msgs::Imu imu_msg;
  double linear_acceleration_stdev, angular_velocity_stdev;
  private_nh.param<std::string>("frame_id", imu_msg.header.frame_id, "imu_link");
  // Defaults obtained experimentally from hardware, no device spec exists
  private_nh.param<double>("linear_acceleration_stdev", linear_acceleration_stdev, (4.0 * 1e-3f * 9.80665));
  private_nh.param<double>("angular_velocity_stdev", angular_velocity_stdev, (0.06 * 3.14159 / 180.0));

  double linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev;
  double angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev;

  // From the UM7 datasheet for the dynamic accuracy from the EKF.
  double orientation_x_stdev, orientation_y_stdev, orientation_z_stdev;
  private_nh.param<double>("orientation_x_stdev", orientation_x_stdev, (3.0 * 3.14159 / 180.0));
  private_nh.param<double>("orientation_y_stdev", orientation_y_stdev, (3.0 * 3.14159 / 180.0));
  private_nh.param<double>("orientation_z_stdev", orientation_z_stdev, (5.0 * 3.14159 / 180.0));

  double orientation_x_covar = orientation_x_stdev * orientation_x_stdev;
  double orientation_y_covar = orientation_y_stdev * orientation_y_stdev;
  double orientation_z_covar = orientation_z_stdev * orientation_z_stdev;

  // Enable converting from NED to ENU by default
  bool tf_ned_to_enu;
  bool orientation_in_robot_frame;
  private_nh.param<bool>("tf_ned_to_enu", tf_ned_to_enu, true);
  private_nh.param<bool>("orientation_in_robot_frame", orientation_in_robot_frame, false);
  OutputAxisOption axes = OutputAxisOptions::DEFAULT;
  if (tf_ned_to_enu && orientation_in_robot_frame)
  {
    ROS_ERROR("Requested IMU data in two separate frames.");
  }
  else if (tf_ned_to_enu)
  {
    axes = OutputAxisOptions::ENU;
  }
  else if (orientation_in_robot_frame)
  {
    axes = OutputAxisOptions::ROBOT_FRAME;
  }

  // Use MagneticField message rather than Vector3Stamped.
  bool use_magnetic_field_msg;
  private_nh.param<bool>("use_magnetic_field_msg", use_magnetic_field_msg, false);

  // These values do not need to be converted
  imu_msg.linear_acceleration_covariance[0] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

  imu_msg.angular_velocity_covariance[0] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[4] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[8] = angular_velocity_cov;

  imu_msg.orientation_covariance[0] = orientation_x_covar;
  imu_msg.orientation_covariance[4] = orientation_y_covar;
  imu_msg.orientation_covariance[8] = orientation_z_covar;

  int32_t dev_baud;
  // findDeviceBaud(ser, user_baud, serial_baud, dev_baud);

  // Real Time Loop
  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.setBaudrate(serial_baud);
      // ROS_WARN("==========================");
      // ROS_WARN("===== TRYING BAUD act: %d, ser_baud:%d , open:%d==", ser.getBaudrate(), serial_baud, (int)ser.isOpen());
      // ros::Duration(1.0).sleep();
      ser.open();
      ser.flush();
      ser.flushInput();
      ser.flushOutput();
      // ROS_WARN("===== SER after opening: act: %d, ser_baud:%d, open:%d", ser.getBaudrate(), serial_baud, (int)ser.isOpen());
      tried_baud_rates.insert(ser.getBaudrate());
    }
    catch (const serial::IOException& e)
    {
        ROS_WARN("um7_driver was unable to connect to port %s, baud: %d.", port.c_str(), serial_baud);
    }
    if (ser.isOpen())
    {
      ROS_WARN("um7_driver successfully connected to serial port %s, baud: %d.", port.c_str(), serial_baud);
      first_failure = true;
      try
      {
        um7::Comms sensor(&ser, (unsigned int)receive_timeout_ms);
        configureSensor(&sensor, &private_nh, user_baud, serial_baud);
        um7::Registers registers;
        ros::ServiceServer srv = imu_nh.advertiseService<um7::Reset::Request, um7::Reset::Response>(
            "reset", boost::bind(handleResetService, &sensor, _1, _2));

        // ros::Time prev_time=ros::Time::now();
        while (ros::ok())
        {
          // triggered by arrival of last message packet
          if (sensor.receive(&registers) == TRIGGER_PACKET)
          {
            // Triggered by arrival of final message in group.
            imu_msg.header.stamp = ros::Time::now();
            publishMsgs(registers, &imu_nh, imu_msg, axes, use_magnetic_field_msg);

            // double tdiff=(imu_msg.header.stamp - prev_time).toSec();
            // double hz=0.0;
            // if (tdiff>0.0){
            //   hz = 1.0/tdiff;
            // }
            // prev_time = imu_msg.header.stamp;
            
            // ROS_WARN("time_now: %f, diff:%f, hz:%f", imu_msg.header.stamp.toSec(), tdiff, hz);

            ros::spinOnce();
          }
        }
      }
      catch(const um7::DeviceWrongBaud& e)
      {
        ROS_ERROR_STREAM(e.what());
        if (ser.isOpen()) ser.close();
        ros::shutdown();
        return -1;
      }
      catch(const um7::DeviceBaudChanged& e)
      {
        // We changed the devices baud to the user_baud
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());

        // Reconnect to serial using the user baud rate
        serial_baud = user_baud;
        ros::Duration(1.0).sleep();
      }
      catch(const std::exception& e)
      {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
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
