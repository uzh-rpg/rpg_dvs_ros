// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <libusb-1.0/libusb.h>
#include <boost/thread.hpp>
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <stdio.h>

namespace dvs {

// DVS USB interface
#define DVS128_VID 0x152A
#define DVS128_PID 0x8400
#define DVS128_DID_TYPE 0x00

#define USB_IO_ENDPOINT 0x86

// DVS data decoding masks
#define DVS128_POLARITY_SHIFT 0
#define DVS128_POLARITY_MASK 0x0001
#define DVS128_Y_ADDR_SHIFT 8
#define DVS128_Y_ADDR_MASK 0x007F
#define DVS128_X_ADDR_SHIFT 1
#define DVS128_X_ADDR_MASK 0x007F
#define DVS128_SYNC_EVENT_MASK 0x8000

// USB vendor requests
#define VENDOR_REQUEST_START_TRANSFER 0xB3
#define VENDOR_REQUEST_STOP_TRANSFER 0xB4
#define VENDOR_REQUEST_SEND_BIASES 0xB8
#define VENDOR_REQUEST_SET_SYNC_ENABLED 0xBE
#define VENDOR_REQUEST_RESET_TIMESTAMPS 0xBB

struct Event {
  uint16_t x, y;
  bool polarity;
  uint64_t timestamp;
};

class DvsDriver {
public:
  DvsDriver(std::string dvs_serial_number = "", bool master = true);
  ~DvsDriver();

  std::vector<Event> getEvents();

  bool changeParameters(uint32_t cas, uint32_t injGnd, uint32_t reqPd, uint32_t puX,
                         uint32_t diffOff, uint32_t req, uint32_t refr, uint32_t puY,
                         uint32_t diffOn, uint32_t diff, uint32_t foll, uint32_t Pr);

  void callback(struct libusb_transfer *transfer);

  void resetTimestamps();

  inline std::string getCameraId() const
  {
    return camera_id_;
  }

  inline bool isDeviceRunning() const
  {
    return device_running_;
  }

private:
  bool changeParameter(std::string parameter, uint32_t value);
  bool sendParameters();

  bool openDevice(std::string dvs_serial_number = "");
  void closeDevice();

  bool device_running_;

  boost::mutex event_buffer_mutex_;
  boost::mutex device_mutex_;
  boost::thread* thread_;
  void run();

  void eventTranslator(uint8_t *buffer, size_t bytes_sent);

  // USB handle and buffer
  libusb_device_handle *device_handle_;
  struct libusb_transfer *transfer_;
  unsigned char *buffer_;

  // event buffer
  std::vector<dvs::Event> event_buffer_;

  // buffers
  static const uint32_t bufferSize = 512;

  uint64_t wrap_add_;
  uint64_t last_timestamp_;

  class Parameter {
  public:
    Parameter(uint32_t min = 0, uint32_t max = 0, uint32_t value = 0) :
      min_(min), max_(max), value_(value) {}

    uint32_t getValue() { return value_; }

    bool setValue(uint32_t value) {
      if (value >= min_ && value <= max_) {
        value_ = value;
        return true;
      }
      else {
        return false;
      }
    }
  private:
    uint32_t min_;
    uint32_t max_;
    uint32_t value_;

  };

  // parameters
  std::map<std::string, Parameter> parameters_;

  // camera name
  std::string camera_id_;

};

} // namespace
