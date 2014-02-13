#ifndef DVS_DRIVER_H_
#define DVS_DRIVER_H_

#include <libusb.h>
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

struct Event {
  uint16_t x, y;
  bool polarity;
  uint32_t timestamp;
};

class DVS_Driver {
public:
  DVS_Driver();
  ~DVS_Driver();

  bool connect();
  void disconnect();

  std::vector<Event> get_events();

  bool change_parameter(std::string parameter, uint32_t value);

  void callback(struct libusb_transfer *transfer);

private:
  bool send_parameters();

  bool open_device();
  void close_device();

  boost::mutex event_buffer_lock;
  boost::thread* thread;
  void run();

  void event_translator(uint8_t *buffer, size_t bytesSent);

  // USB handle and buffer
  libusb_device_handle *device_handle;
  struct libusb_transfer *transfer;
  unsigned char *buffer;

  // event buffer
  std::vector<dvs::Event> event_buffer;

  // parameters
  std::map<std::string, uint32_t> parameters;

  // buffers
  static const uint32_t bufferNumber = 8;
  static const uint32_t bufferSize = 4096;

  uint32_t wrapAdd;
  uint32_t lastTimestamp;
};

} // namespace
#endif
