#ifndef DVS_DRIVER_H_
#define DVS_DRIVER_H_

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

class DVS_Driver {
public:
  DVS_Driver(std::string dvs_serial_number = "", bool master = true);
  ~DVS_Driver();

  std::vector<Event> get_events();

  bool change_parameters(uint32_t cas, uint32_t injGnd, uint32_t reqPd, uint32_t puX,
                         uint32_t diffOff, uint32_t req, uint32_t refr, uint32_t puY,
                         uint32_t diffOn, uint32_t diff, uint32_t foll, uint32_t pr);

  void callback(struct libusb_transfer *transfer);

  void resetTimestamps();

  inline std::string get_camera_id() {
    return camera_id;
  }

private:
  bool change_parameter(std::string parameter, uint32_t value);
  bool send_parameters();

  bool open_device(std::string dvs_serial_number = "");
  void close_device();

  boost::mutex event_buffer_mutex;
  boost::mutex device_mutex;
  boost::thread* thread;
  void run();

  void event_translator(uint8_t *buffer, size_t bytesSent);

  // USB handle and buffer
  libusb_device_handle *device_handle;
  struct libusb_transfer *transfer;
  unsigned char *buffer;

  // event buffer
  std::vector<dvs::Event> event_buffer;

  // buffers
  static const uint32_t bufferNumber = 8;
  static const uint32_t bufferSize = 4096;

  uint64_t wrapAdd;
  uint64_t lastTimestamp;

  class Parameter {
  public:
    Parameter(uint32_t min = 0, uint32_t max = 0, uint32_t value = 0) :
      _min(min), _max(max), _value(value) {}

    uint32_t get_value() { return _value; }

    bool set_value(uint32_t value) {
      if (value >= _min && value <= _max) {
        _value = value;
        return true;
      }
      else {
        return false;
      }
    }
  private:
    uint32_t _min;
    uint32_t _max;
    uint32_t _value;

  };

  // parameters
  std::map<std::string, Parameter> parameters;

  // camera name
  std::string camera_id;
};

} // namespace
#endif
