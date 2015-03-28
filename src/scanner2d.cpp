/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, John Jordan
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rhoeby Dynamics nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "scanner2d.h"

#define SCANNER2D_USE_CONSOLE 1

namespace scanner2d
{

#define SCANNER2D_CMD_LEN_MAX           32
#define SCANNER2D_MSG_LEN_MAX           1024
#define SCANNER2D_MSG_HEADER_LEN        5
#define SCANNER2D_READ_LOOP_MAX         256 // maximum number of consecutive bytes to read from the port 

typedef enum Scanner2dCommands_t
{
  SCANNER2D_CMD_START,
  SCANNER2D_CMD_STOP,
  SCANNER2D_CMD_SET_SCAN_PERIOD,
  SCANNER2D_CMD_RESET,
  SCANNER2D_CMD_CLEAR_RESET,
  SCANNER2D_CMD_MAX,
} Scanner2dCommands_t;

typedef struct Scanner2dMsg_t
{
  uint8_t msg_type;
  uint16_t payload_length;
  uint8_t buffer[SCANNER2D_MSG_LEN_MAX];

  uint8_t parser_state;
  uint16_t payload_byte_count;
} Scanner2dMsg_t;

void consoleSetTerminalMode();
char processConsole(Scanner2d *pScanner);

/*----------------------------------------------------------
 * Constructor()
 *--------------------------------------------------------*/

Scanner2d::Scanner2d()
{
  msg_ = new Scanner2dMsg_t;
  command_buffer_ = new uint8_t[SCANNER2D_CMD_LEN_MAX];

  memset(msg_, 0, sizeof(Scanner2dMsg_t));
  memset(&status_, 0, sizeof(status_));
  status_.samples_per_scan = 200;
  status_.scan_period = 250;

  port_fd_ = -1;

  thread_ = boost::thread(&Scanner2d::processThread, this);
}

/*----------------------------------------------------------
 * Destructor()
 *--------------------------------------------------------*/

Scanner2d::~Scanner2d()
{
  close();

  thread_.interrupt();
  thread_.join();

  delete command_buffer_;
  delete msg_;
}

/*----------------------------------------------------------
 * open() - open the port
 *--------------------------------------------------------*/

void Scanner2d::open(const std::string port_name)
{
  struct termios new_termios;

  port_fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (port_fd_ == -1)
  {
    ROS_ERROR("%s open failed!", port_name.c_str());
    exit(-1);
  }
  else
  {
    fcntl(port_fd_, F_SETFL, 0);
  }

  tcgetattr(port_fd_, &new_termios);
  cfmakeraw(&new_termios);
  cfsetospeed(&new_termios, B115200);
  tcsetattr(port_fd_, TCSANOW, &new_termios);

  if (msgDataWaiting())
  {
    ROS_DEBUG("Clearing Rx buffer!");
    while (msgDataWaiting())
    {
      ::read(port_fd_, &msg_->buffer, SCANNER2D_MSG_LEN_MAX-1);
    }
    ROS_DEBUG("Rx buffer cleared");
  }
  else
  {
    ROS_DEBUG("No messages to clear in Rx buffer!");
  }

  reset();

#if SCANNER2D_USE_CONSOLE
  consoleSetTerminalMode();
#endif
}

/*----------------------------------------------------------
 * close() - close the port
 *--------------------------------------------------------*/

void Scanner2d::close()
{
  int rc;

  if (port_fd_ != -1)
  {
    rc = ::close(port_fd_);
    if (rc) ROS_ERROR("close failed!");
  }

  port_fd_ = -1;
}

/*----------------------------------------------------------
 * calcChecksum() - calculate the checksum
 *--------------------------------------------------------*/

uint8_t Scanner2d::calcChecksum(const uint8_t *data, const uint16_t length)
{
  uint32_t i;
  uint8_t checksum = 0;

  for (i=0; i<length; i++)
  {
    checksum += data[i];
  }

  return ~checksum;
}

/*----------------------------------------------------------
 * start() - start the scanner 
 *--------------------------------------------------------*/

void Scanner2d::start()
{
  ssize_t msg_length = 5;
  ssize_t bytes_written;

  command_buffer_[0] = 0xFF;
  command_buffer_[1] = 0xFF;
  command_buffer_[2] = SCANNER2D_CMD_START;
  command_buffer_[3] = 0; // payload length
  command_buffer_[4] = calcChecksum(command_buffer_, msg_length-1);

  bytes_written = ::write(port_fd_, command_buffer_, msg_length);
  if (bytes_written != msg_length)
  {
    ROS_ERROR("start command failed!");
  }
}

/*----------------------------------------------------------
 * stop() - stop the scanner 
 *--------------------------------------------------------*/

void Scanner2d::stop()
{
  ssize_t msg_length = 5;
  ssize_t bytes_written;

  command_buffer_[0] = 0xFF;
  command_buffer_[1] = 0xFF;
  command_buffer_[2] = SCANNER2D_CMD_STOP;
  command_buffer_[3] = 0; // payload length
  command_buffer_[4] = calcChecksum(command_buffer_, msg_length-1);

  bytes_written = ::write(port_fd_, command_buffer_, msg_length);
  if (bytes_written != msg_length)
  {
    ROS_ERROR("stop command failed!");
  }
}

/*----------------------------------------------------------
 * reset() - reset the scanner 
 *--------------------------------------------------------*/
void Scanner2d::reset()
{
  ssize_t msg_length = 5;
  ssize_t bytes_written;
  uint32_t count;
  uint32_t retry_count;

  command_buffer_[0] = 0xFF;
  command_buffer_[1] = 0xFF;
  command_buffer_[2] = SCANNER2D_CMD_CLEAR_RESET;
  command_buffer_[3] = 0; // payload length
  command_buffer_[4] = calcChecksum(command_buffer_, msg_length-1);

  retry_count = 0;

  retry_clear:
  bytes_written = ::write(port_fd_, command_buffer_, msg_length);
  if (bytes_written != msg_length)
  {
    ROS_ERROR("reset CLEAR command write failed!");
  }

  ROS_DEBUG("Waiting for reset CLEAR to complete...");
  count = 0;
  while (status_.flags & 0x01)
  {
    process();

    count++;
    if (count >= 100)
    {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  if (count >= 100)
  {
    retry_count++;
    if (retry_count < 5)
    {
      goto retry_clear;
    }
    else
    {
      ROS_WARN("Reset CLEAR complete timeout");
    }
  }
  else
  {
    ROS_DEBUG("Reset CLEAR complete");
  }

  command_buffer_[0] = 0xFF;
  command_buffer_[1] = 0xFF;
  command_buffer_[2] = SCANNER2D_CMD_RESET;
  command_buffer_[3] = 0; // payload length
  command_buffer_[4] = calcChecksum(command_buffer_, msg_length-1);

  bytes_written = ::write(port_fd_, command_buffer_, msg_length);
  if (bytes_written != msg_length)
  {
    ROS_ERROR("reset command failed!");
  }

  ROS_DEBUG("Waiting for reset to complete...");
  count = 0;
  while ((!(status_.flags & 0x01)) && count<1000)
  {
    ros::Duration(0.01).sleep();
    process();
    count++;
  }

  ros::Duration(0.5).sleep();

  if (count>=1000)
  {
    ROS_WARN("Reset complete timeout");
  }
  else
  {
    ROS_DEBUG("Reset complete");
  }
}

/*----------------------------------------------------------
 * setScanPeriod() - set the scan period (in milli-seconds) 
 *--------------------------------------------------------*/

void Scanner2d::setScanPeriod(const uint16_t period)
{
  ssize_t msg_length = 7;
  ssize_t bytes_written;

  ROS_DEBUG("Sending setScanPeriod: %d command", period);

  command_buffer_[0] = 0xFF;
  command_buffer_[1] = 0xFF;
  command_buffer_[2] = SCANNER2D_CMD_SET_SCAN_PERIOD;
  command_buffer_[3] = 2; // payload length
  command_buffer_[4] = (period & 0xFF);
  command_buffer_[5] = (period & 0xFF00) >> 8;
  command_buffer_[6] = calcChecksum(command_buffer_, msg_length-1);

  bytes_written = ::write(port_fd_, command_buffer_, msg_length);
  if (bytes_written != msg_length)
  {
    ROS_ERROR("setScanPeriod command failed!");
  }
}

/*----------------------------------------------------------
 * getStatus() - returns the current status of the scanner 
 *--------------------------------------------------------*/

void Scanner2d::getStatus(Scanner2dStatus_t *status)
{
  memcpy(status, &status_, sizeof(status_));
}

/*------------------------------------------------------------
 * processThread() - thread method
 *----------------------------------------------------------*/

void Scanner2d::processThread()
{
  while (ros::ok())
  {
    process();

#if SCANNER2D_USE_CONSOLE
    if (processConsole(this) == 'q')
    {
      ROS_DEBUG("Got 'q' back from processConsole()!");
      close();

      status_.flags |= 0x80; // signal the node we want to quit
      break;
    }
#endif 

    ros::Duration(0.01).sleep();
  }
}

/*------------------------------------------------------------
 * process() - process data from the scanner 
 *----------------------------------------------------------*/

void Scanner2d::process()
{
  loop_count_ = 0;
  uint8_t msg_byte;

  if (port_fd_ == -1)
  {
    return;
  }

  while (msgDataWaiting() && loop_count_ < SCANNER2D_READ_LOOP_MAX)
  {
    ::read(port_fd_, &msg_byte, 1);
    processMsgData(msg_byte);
    loop_count_++;
  }
}

/*----------------------------------------------------------
 * processMsgData() - read data from the scanner 
 *--------------------------------------------------------*/

void Scanner2d::processMsgData(const uint8_t msg_byte)
{
  uint8_t checksum;

  switch (msg_->parser_state)
  {
  case 0:
    msg_->buffer[0] = msg_byte;
    if (msg_byte == 0xFF)
      msg_->parser_state = 1;
    break;
  case 1:
    msg_->buffer[1] = msg_byte;
    if (msg_byte == 0xFF)
      msg_->parser_state = 2;
    else
      msg_->parser_state = 0;
    break;
  case 2:
    msg_->buffer[2] = msg_byte;
    msg_->msg_type = msg_byte;
    msg_->parser_state = 3;
    break;
  case 3:
    msg_->buffer[3] = msg_byte;
    msg_->payload_length = (msg_byte << 8);
    msg_->parser_state = 4;
    break;
  case 4:
    msg_->buffer[4] = msg_byte;
    msg_->payload_length |= msg_byte;
    ROS_DEBUG("msg_->payload_length: %d", msg_->payload_length);

    if (msg_->payload_length >= SCANNER2D_MSG_LEN_MAX)
    {
      ROS_WARN("Got bad payload length in message from scanner!");
      ROS_WARN("  msg_->payload_length: %d", msg_->payload_length);
      msg_->parser_state = 0;
    }
    else if (msg_->payload_length)
    {
      msg_->payload_byte_count = 0;
      msg_->parser_state = 5;
    }
    else
    {
      msg_->parser_state = 6;
    }
    break;
  case 5:
    msg_->buffer[msg_->payload_byte_count + 5] = msg_byte;
    msg_->payload_byte_count++;
    if (msg_->payload_byte_count + 5 > SCANNER2D_MSG_LEN_MAX)
    {
      ROS_DEBUG("Got bad payload byte count!");
      ROS_DEBUG("  msg_->payload_byte_count: %d", msg_->payload_byte_count);
      msg_->parser_state = 0;
    }
    else if (msg_->payload_byte_count >= msg_->payload_length)
    {
      msg_->parser_state = 6;
    }
    break;
  case 6:
    checksum = calcChecksum(msg_->buffer, msg_->payload_byte_count + 5);
    if (msg_byte == checksum)
    {
      ROS_DEBUG("Got complete message (type: %d)!", msg_->msg_type);
      processMsg();
    }
    else
    {
      ROS_DEBUG("Message (type: %d) checksum failed!", msg_->msg_type);
    }

    msg_->parser_state = 0;
    break;
  default:
    ROS_WARN("Bad parser state!");
    msg_->parser_state = 0;
    break;
  }
}

/*----------------------------------------------------------
 * processMsg() - process message from the scanner 
 *--------------------------------------------------------*/

void Scanner2d::processMsg()
{
  uint8_t *payload = &(msg_->buffer[SCANNER2D_MSG_HEADER_LEN]);

  switch (msg_->msg_type)
  {
  case 0:
    publishScan(payload, msg_->payload_length);
    break;
  case 1:
    updateStatus(payload);
    break;
  }
}

/*----------------------------------------------------------
 * updateStatus() - process status message from the scanner 
 *--------------------------------------------------------*/

void Scanner2d::updateStatus(const uint8_t *buffer)
{
  status_.samples_per_scan = buffer[0] | (buffer[1] << 8);
  status_.scan_period = buffer[2] | (buffer[3] << 8);
  status_.flags = buffer[4];

  printStatus();
}

/*----------------------------------------------------------
 * printStatus() - print status message from the scanner 
 *--------------------------------------------------------*/

void Scanner2d::printStatus(void)
{
  ROS_DEBUG("Scanner status:");
  ROS_DEBUG("  samples_per_scan: %d", status_.samples_per_scan);
  ROS_DEBUG("  scan_period: %d", status_.scan_period);
  ROS_DEBUG("  statusFlags: %d", status_.flags);
}

/*----------------------------------------------------------
 * publish() - publish scan data from the scanner 
 *--------------------------------------------------------*/

void Scanner2d::publishScan(const uint8_t *buffer, const uint16_t len)
{
  uint32_t i;
  static ros::Publisher laser_pub = node_.advertise<sensor_msgs::LaserScan>("laser_data", 20);
  ros::Time current_time = ros::Time::now();
  uint16_t samples_per_scan = len / 2;
  sensor_msgs::LaserScan scan;

  scan.header.frame_id = "base_laser";

  scan.scan_time = status_.scan_period / 1000.0;
  scan.range_min = 0.1;
  scan.range_max = 3.6576;

  scan.header.stamp = current_time - ros::Duration(scan.scan_time);
  scan.angle_max = 3.142;
  scan.angle_min = -scan.angle_max;

  scan.time_increment = scan.scan_time / samples_per_scan;
  scan.angle_increment = 2 * 3.142 / samples_per_scan;
  scan.ranges.resize(samples_per_scan);
  for (i=0; i<len; i+=2)
  {
    scan.ranges[i / 2] = (buffer[i] | (buffer[i+1] << 8)) / 1000.0;
  }

  laser_pub.publish(scan);

  ROS_DEBUG("Published scan");
}

/*---------------------------------------------------------------
 * msgDataWaiting() - check if data has arrived from the scanner 
 *-------------------------------------------------------------*/

bool Scanner2d::msgDataWaiting()
{
  struct timeval tv = { 0L, 0L};
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(port_fd_, &fds);

  select(port_fd_+1, &fds, NULL, NULL, &tv);

  return FD_ISSET(port_fd_, &fds);
}

/*---------------------------------------------------------------
 * console functions 
 *-------------------------------------------------------------*/

#if SCANNER2D_USE_CONSOLE
struct termios orig_termios;

void consoleResetTerminalMode();
int consoleKbHit();
int consoleGetCh();

static bool started = true;

char processConsole(Scanner2d *pScanner)
{
  char kbd_ch = 0;
  int i;

  if (consoleKbHit())
  {
    kbd_ch = consoleGetCh();
    switch (kbd_ch)
    {
    case '1':
      pScanner->setScanPeriod(1000);
      break;
    case '2':
      pScanner->setScanPeriod(500);
      break;
    case '4':
      pScanner->setScanPeriod(250);
      break;    
    case '5':
      pScanner->setScanPeriod(200);
      break;    
    case 'P':
      if (started)
      {
        pScanner->stop();
        started = false;
      }
      else
      {
        pScanner->start();
        started = true;
      }
      break;
    case 'R':
      pScanner->reset();
      break;
    case 'S':
      pScanner->printStatus();
      break;
    default:
      break;
    }
  }

  return kbd_ch;
}

void consoleResetTerminalMode()
{
  tcsetattr(0, TCSANOW, &orig_termios);
}

void consoleSetTerminalMode()
{
  struct termios new_termios;

  /* take two copies - one for now, one for later */
  tcgetattr(0, &orig_termios);
  memcpy(&new_termios, &orig_termios, sizeof(new_termios));

  /* register cleanup handler, and set the new terminal mode */
  atexit(consoleResetTerminalMode);

  cfmakeraw(&new_termios);

  new_termios.c_oflag |= OPOST;

  tcsetattr(0, TCSANOW, &new_termios);
}

int consoleKbHit()
{
  struct timeval tv = { 0L, 0L};

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

int consoleGetCh()
{
  int r;
  unsigned char c;
  if ((r = read(0, &c, sizeof(c))) < 0)
  {
    return r;
  }
  else
  {
    return c;
  }
}
#endif

} // namespace scanner2d
