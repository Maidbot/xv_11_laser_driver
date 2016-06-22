/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *  Copyright (c) 2016, Maidbot, Inc.
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
 *   * Neither the name of Case Western Reserve University nor the names of its
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
 *********************************************************************/

#include <xv_11_laser_driver/xv11_laser.h>

namespace xv_11_laser_driver {

  XV11Laser::XV11Laser(const std::string& port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service& io): port_(port),
  baud_rate_(baud_rate), firmware_(firmware), shutting_down_(false), serial_(io, port_) {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  }

  void XV11Laser::poll(sensor_msgs::LaserScan::Ptr scan) {

    uint8_t start_count = 0;
    bool got_scan = false;

    if(firmware_ == 1){ // This is for the old driver, the one that only outputs speed once per revolution
      uint8_t temp_char;
      boost::array<uint8_t, 1440> raw_bytes;
      while (!shutting_down_ && !got_scan) {
    // Wait until the start sequence 0x5A, 0xA5, 0x00, 0xC0 comes around
    boost::asio::read(serial_, boost::asio::buffer(&temp_char,1));
    if(start_count == 0) {
      if(temp_char == 0x5A) {
        start_count = 1;
      }
    } else if(start_count == 1) {
      if(temp_char == 0xA5) {
        start_count = 2;
      }
    } else if(start_count == 2) {
      if(temp_char == 0x00) {
        start_count = 3;
      }
    } else if(start_count == 3) {
      if(temp_char == 0xC0) {
        start_count = 0;
        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;
        // Now read speed
        boost::asio::read(serial_,boost::asio::buffer(&motor_speed_,2));

        // Read in 360*4 = 1440 chars for each point
        boost::asio::read(serial_,boost::asio::buffer(&raw_bytes,1440));

        scan->angle_min = 0.0;
        scan->angle_max = 2.0*M_PI;
        scan->angle_increment = (2.0*M_PI/360.0);
        scan->time_increment = motor_speed_/1e8;
        scan->range_min = 0.06;
        scan->range_max = 5.0;
        scan->ranges.reserve(360);
        scan->intensities.reserve(360);

        for(uint16_t i = 0; i < raw_bytes.size(); i=i+4) {
          // Four bytes per reading
          uint8_t byte0 = raw_bytes[i];
          uint8_t byte1 = raw_bytes[i+1];
          uint8_t byte2 = raw_bytes[i+2];
          uint8_t byte3 = raw_bytes[i+3];
          // First two bits of byte1 are status flags
          uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
          uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
          // Remaining bits are the range in mm
          uint16_t range = ((byte1 & 0x3F)<< 8) + byte0;
          // Last two bytes represent the uncertainty or intensity, might also be pixel area of target...
          uint16_t intensity = (byte3 << 8) + byte2;

          scan->ranges.push_back(range / 1000.0);
          scan->intensities.push_back(intensity);
        }
      }
    }
      }

    // This is for the newer firmware that outputs packets with 4 readings each
    } else if (firmware_ == 2) {

      const float ONE_DEGREE = 2.0 * M_PI / 360.0;

      scan->angle_min = 0.0;
      scan->angle_max = 2.0 * M_PI - ONE_DEGREE; // No double-count
      scan->angle_increment = ONE_DEGREE;
      scan->range_min = 0.15;
      scan->range_max = 5.0;
      scan->ranges.resize(360);
      scan->intensities.resize(360);

      const uint8_t HEADER_BYTE = 0xFA;
      const uint8_t FIRST_INDEX_BYTE = 0xA0;

      boost::array<uint8_t, 1980> raw_bytes;

      int angle; // The index of the scan.ranges array. Ranges from 0 to 359
      uint8_t packet_index;     // Second byte of a packet. Ranges from 0 to 89
      bool good_packet;
      uint8_t good_packets = 0; // Number of good packets. Ranges from 0 to 90.
      int rpms_sum = 0;    // For calculating average motor speed
      rpms = 0;

      while (!shutting_down_ && !got_scan) {

        // Wait until first data sync of frame, i.e., 0xFA, 0xA0
        boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));

        if (start_count == 0) {
          if (raw_bytes[start_count] == HEADER_BYTE) {
            start_count = 1;
          }
        } else if (start_count == 1) {
          if (raw_bytes[start_count] == FIRST_INDEX_BYTE) {
            start_count = 0; // NOTE: Is this actually useful?

            // Now that the start sequence has been found (0xFA, 0xA0),
            // start reading packets until the laser scan is complete
            got_scan = true;

            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2],
                                                        raw_bytes.size() - 2));

            uint16_t i = 0; // Iterates over the raw byte stream

            while (!shutting_down_ && (i + 22 <= raw_bytes.size())) {

              packet_index = (raw_bytes[i+1] - FIRST_INDEX_BYTE);

              if (raw_bytes[i] == HEADER_BYTE && packet_index >= 0
                                              && packet_index < 90) {

                // Check whether this seems to be a proper packet of length 22
                good_packet = true; // Start optimistically

                for (uint16_t k = 2; k < 22; k++) {

                  // If there is a premature header byte, skip this packet
                  if (raw_bytes[i+k] == HEADER_BYTE) {
                    good_packet = false;
                    i = i + k;
                    break;
                  }
                }

                if (good_packet) {
                  // TODO: Add CRC checksum too before declaring good packet

                  good_packets++;
                  // std::cout << "Good packet starting at i = " << i << "\n";

                  rpms_sum += (raw_bytes[i+3]<<8 | raw_bytes[i+2]) / 64;

                  // Iterate over the 4 measurements of this packet
                  for (uint16_t j = i+4; j < i+20; j=j+4) {

                    // Calculate the bearing angle (index of ranges)
                    angle = (4 * packet_index) + (j-4-i)/4;

                    // Four bytes per measurement
                    uint8_t byte0 = raw_bytes[j];
                    uint8_t byte1 = raw_bytes[j+1];
                    uint8_t byte2 = raw_bytes[j+2];
                    uint8_t byte3 = raw_bytes[j+3];

                    // The first two bits of byte1 are status flags
                    // No return/max range/too low of reflectivity
                    // uint8_t flag1 = (byte1 & 0x80) >> 7;
                    // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                    // uint8_t flag2 = (byte1 & 0x40) >> 6;

                    // Remaining bits are the range in mm
                    uint16_t range = ((byte1 & 0x3F)<< 8) + byte0;

                    // Last two bytes represent the uncertainty or
                    // intensity, might also be pixel area of target
                    uint16_t intensity = (byte3 << 8) + byte2;

                    scan->ranges[angle] = range / 1000.0;
                    scan->intensities[angle] = intensity;
                  }

                  if (angle == 359) {
                    break; // Laser scan message is full
                  } else {
                    i = i + 22; // Set iteration index to start of next packet
                  }

                // } else {
                //   std::cout << "Bad packet starting at i = " << i - k
                //             << " Found unexpected header byte at i = " << i
                //             << "\n";

                } // End of packet length (and eventually CRC) check

              } else {
                i++;
              } // End of start of packet check
            } // End of inner while loop

            // std::cout << "<-- Good packets for this revolution = "
            //           << static_cast<int>(good_packets) << " / 90 -->\n";

            if (rpms_sum > 0 && good_packets > 0) {
                rpms = rpms_sum / good_packets;
                scan->scan_time = 60.0 / rpms;
            } else {
                rpms = 0.0;
                scan->scan_time = 0.0;
            }

            scan->time_increment = scan->scan_time / 360.0;

          } else {
            start_count = 0;
          } // End of check for frame sync, i.e., 0xFA, 0xA0
        } //  Same^
      } // End of outer while loop
    } // End of firmware version 2 case
  }
};
