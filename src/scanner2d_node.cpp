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

#include <string>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "scanner2d.h"

static void sig_handler(int s);

static bool got_ctrl_c = false;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scanner2d");
  scanner2d::Scanner2d scanner;
  std::string port_name;
  int scan_rate;
  int sample_rejection;
  int samples_per_scan;
  int min_angle;
  int max_angle;

  ROS_DEBUG("Welcome to scanner2d Node!");

  signal(SIGINT, sig_handler);
  
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("port_name", port_name, std::string("/dev/ttyACM0"));
  private_node_handle_.param("scan_rate", scan_rate, int(3));
  private_node_handle_.param("samples_per_scan", samples_per_scan, int(333));
  private_node_handle_.param("sample_rejection", sample_rejection, int(0));
  private_node_handle_.param("min_angle", min_angle, int(0));
  private_node_handle_.param("max_angle", max_angle, int(360));

  if (scan_rate*samples_per_scan > 1000) {
    ROS_WARN("The scan_rate * samples_per_scan exceeds the max sample rate (1000) of the sensor!");
    ROS_WARN("  you should either alter the settings from the command line invocation,");
    ROS_WARN("  or reconfigure the parameter server directly.");
  }

  scanner.open(port_name);

  switch(scan_rate)
  {
  case 1:
    scanner.setScanPeriod(1000);
    break;
  case 2:
    scanner.setScanPeriod(500);
    break;
  case 3:
    scanner.setScanPeriod(333);
    break;
  case 4:
    scanner.setScanPeriod(250);
    break;
  case 5:
    scanner.setScanPeriod(200);
    break;
  default:
    ROS_WARN("Invalid scan_rate!");
  }
  
  scanner.setSampleRejectionMode((bool)sample_rejection);
  scanner.setSamplesPerScan(samples_per_scan);
  scanner.setMinMaxAngle(min_angle,  max_angle);

  ros::Rate loop_rate(10);

  while (ros::ok()) 
  {
    scanner2d::Scanner2dStatus_t status;
    scanner.getStatus(&status);
    if (status.flags & 0x80) 
    {
      ROS_DEBUG("Got quit from scanner");
      ROS_DEBUG("  Status flags: 0x%x", status.flags);
      break;
    }

    if (got_ctrl_c) {
      ROS_WARN("Got Ctrl-C");
      scanner.stop();
      ros::Duration(0.5).sleep();
      scanner.close();
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_WARN("Exiting.");

  exit(0);
}

void sig_handler(int s)
{
  if (s == 2) {
    got_ctrl_c = true; 
  }
}

