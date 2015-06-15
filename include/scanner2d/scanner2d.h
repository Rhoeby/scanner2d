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

#ifndef _SCANNER2D_NODE_
#define _SCANNER2D_NODE_

#include <boost/thread.hpp>

namespace scanner2d
{

using std::string;

typedef struct Scanner2dStatus_t
{
  uint16_t samples_per_scan;
  uint16_t scan_period;
  uint8_t flags;
  uint16_t range_reading;
} Scanner2dStatus_t;

struct Scanner2dMsg_t;

class Scanner2d
{
public:
  Scanner2d();
  virtual ~Scanner2d();

  void open(const std::string port_name);
  void close();
  void start();
  void stop();

  void getStatus(Scanner2dStatus_t *status);
  void setScanPeriod(const uint16_t period);
  void setSamplesPerScan(const uint16_t samples_per_scan);
  void setSampleRejectionMode(const bool enabled);
  void setParkTrim(const int16_t trim);
  void setMinMaxAngle(const uint16_t min, const uint16_t max);

private:
  void processThread();
  void process();
  
  bool msgDataWaiting();
  void processMsgData(const uint8_t msg_byte);
  void processMsg();
  uint8_t calcChecksum(const uint8_t *data, const uint16_t length);

  void publishScan(const uint8_t *buffer, const uint16_t len);
  void updateStatus(const uint8_t *buffer);
  void printStatus(void);

  void reset(); // can block for a long time!
  friend char processConsole(Scanner2d *pScanner);

  int32_t port_fd_;
  uint32_t loop_count_;
  
  uint8_t *command_buffer_;
  Scanner2dMsg_t *msg_;
  Scanner2dStatus_t status_;

  ros::NodeHandle node_;

  boost::thread thread_;
};

} // namespace scanner2d
#endif
