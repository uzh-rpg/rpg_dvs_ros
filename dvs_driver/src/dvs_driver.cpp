#include "dvs_driver/dvs_driver.h"

namespace dvs {

extern "C" { 

static void callback_wrapper(struct libusb_transfer *transfer) 
{ 
  DVS_Driver *dvs_driver = (DVS_Driver *)transfer->user_data;

  dvs_driver->callback(transfer);
} 

} // extern "C"

DVS_Driver::DVS_Driver() {
  // initialize parameters
  parameters.insert(std::pair<std::string, uint32_t>("cas", 1992));
  parameters.insert(std::pair<std::string, uint32_t>("injGnd", 1108364));
  parameters.insert(std::pair<std::string, uint32_t>("reqPd", 16777215));
  parameters.insert(std::pair<std::string, uint32_t>("puX", 8159221));
  parameters.insert(std::pair<std::string, uint32_t>("diffOff", 132));
  parameters.insert(std::pair<std::string, uint32_t>("req", 309590));
  parameters.insert(std::pair<std::string, uint32_t>("refr", 969));
  parameters.insert(std::pair<std::string, uint32_t>("puY", 16777215));
  parameters.insert(std::pair<std::string, uint32_t>("diffOn", 209996));
  parameters.insert(std::pair<std::string, uint32_t>("diff", 13125));
  parameters.insert(std::pair<std::string, uint32_t>("foll", 271));
  parameters.insert(std::pair<std::string, uint32_t>("pr", 217));

  libusb_init(NULL);

  device_mutex.lock();
  open_device();
  device_mutex.unlock();

  thread = new boost::thread(boost::bind(&DVS_Driver::run, this));
}

DVS_Driver::~DVS_Driver() {
  device_mutex.lock();
  close_device();
  device_mutex.unlock();
}

bool DVS_Driver::open_device() {
  // try to open device
  device_handle = libusb_open_device_with_vid_pid(NULL, DVS128_VID, DVS128_PID);
  if (device_handle == NULL)
    return false;

// libusb_set_auto_detach_kernel_driver(device_handle, 1);
  libusb_claim_interface(device_handle, 0);

  transfer = libusb_alloc_transfer(0);

  transfer->length = (int) bufferSize;
  buffer = new unsigned char[bufferSize];
  transfer->buffer = buffer;

  transfer->dev_handle = device_handle;
  transfer->endpoint = USB_IO_ENDPOINT;
  transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
  transfer->callback = callback_wrapper;
  transfer->user_data = this;
  transfer->timeout = 0;
  transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

//  libusb_submit_transfer(transfer);
  int status = libusb_submit_transfer(transfer);
  if (status != LIBUSB_SUCCESS) {
    std::cout << "Unable to submit libusb transfer: " << status << std::endl;

    // The transfer buffer is freed automatically here thanks to
    // the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
    libusb_free_transfer(transfer);
  }

  std::cout << "DVS128: data acquisition thread ready to process events." << std::endl;

  libusb_control_transfer(device_handle,
                          LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0, 0);
}

void DVS_Driver::close_device() {
  // stop thread
  thread->join();

  // close transfer
  int status = libusb_cancel_transfer(transfer);
  if (status != LIBUSB_SUCCESS && status != LIBUSB_ERROR_NOT_FOUND) {
    std::cout << "Unable to cancel libusb transfer: " << status << std::endl;
  }

  libusb_close(device_handle);
  libusb_exit(NULL);
}

void DVS_Driver::run() {
  timeval te;
  te.tv_sec = 0;
  te.tv_usec = 1000000;

  try {
    int completed;
    while(true) {
      device_mutex.lock();
      libusb_handle_events_timeout_completed(NULL, &te, &completed);
      device_mutex.unlock();
    }
  }
  catch(boost::thread_interrupted const& ) {
    //clean resources
    std::cout << "Worker thread interrupted" << std::endl;
  }
}

void DVS_Driver::callback(struct libusb_transfer *transfer) {
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    // Handle data.
    event_buffer_mutex.lock();
    event_translator(transfer->buffer, (size_t) transfer->actual_length);
    event_buffer_mutex.unlock();
  }

  if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
    // Submit transfer again.
    if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
      return;
    }
  }

  // Cannot recover (cancelled, no device, or other critical error).
  // Signal this by adjusting the counter, free and exit.
  libusb_free_transfer(transfer);
}

void DVS_Driver::event_translator(uint8_t *buffer, size_t bytesSent) {
  // Truncate off any extra partial event.
  bytesSent &= (size_t) ~0x03;

  for (size_t i = 0; i < bytesSent; i += 4) {
    if ((buffer[i + 3] & 0x80) == 0x80) {
      // timestamp bit 15 is one -> wrap: now we need to increment
      // the wrapAdd, uses only 14 bit timestamps
      wrapAdd += 0x4000;

      // Detect big timestamp wrap-around.
      if (wrapAdd == 0) {
        // Reset lastTimestamp to zero at this point, so we can again
        // start detecting overruns of the 32bit value.
        lastTimestamp = 0;
      }
    }
    else if ((buffer[i + 3] & 0x40) == 0x40) {
      // timestamp bit 14 is one -> wrapAdd reset: this firmware
      // version uses reset events to reset timestamps
      wrapAdd = 0;
      lastTimestamp = 0;
    }
    else {
      // address is LSB MSB (USB is LE)
      uint16_t addressUSB = le16toh(*((uint16_t * ) (&buffer[i])));

      // same for timestamp, LSB MSB (USB is LE)
      // 15 bit value of timestamp in 1 us tick
      uint16_t timestampUSB = le16toh(*((uint16_t * ) (&buffer[i + 2])));

      // Expand to 32 bits. (Tick is 1µs already.)
      uint32_t timestamp = timestampUSB + wrapAdd;

      // Check monotonicity of timestamps.
      if (timestamp < lastTimestamp) {
        // std::cout << "DVS128: non-monotonic time-stamp detected: lastTimestamp=" << lastTimestamp << ", timestamp=" << timestamp << "." << std::endl;
      }

      lastTimestamp = timestamp;

      if ((addressUSB & DVS128_SYNC_EVENT_MASK) != 0) {
        // Special Trigger Event (MSB is set)
      }
      else {
        // Invert x values (flip along the x axis).
        uint16_t x = (uint16_t) (127 - ((uint16_t) ((addressUSB >> DVS128_X_ADDR_SHIFT) & DVS128_X_ADDR_MASK)));
        uint16_t y = (uint16_t) (127 - ((uint16_t) ((addressUSB >> DVS128_Y_ADDR_SHIFT) & DVS128_Y_ADDR_MASK)));
        bool polarity = (((addressUSB >> DVS128_POLARITY_SHIFT) & DVS128_POLARITY_MASK) == 0) ? (1) : (0);

//        std::cout << "Event: <x, y, t, p> = <" << x << ", " << y << ", " << timestamp << ", " << polarity << ">" << std::endl;
        Event e;
        e.x = x;
        e.y = y;
        e.timestamp = timestamp;
        e.polarity = polarity;
        event_buffer.push_back(e);
      }
    }
  }
}

std::vector<Event> DVS_Driver::get_events() {
  event_buffer_mutex.lock();
  std::vector<Event> test = event_buffer;
  event_buffer.clear();
  event_buffer_mutex.unlock();
  return test;
}

bool DVS_Driver::change_parameter(std::string parameter, uint32_t value) {
  if (parameters.find(parameter) != parameters.end()) {
    parameters[parameter] = value;
    return send_parameters();
  } else {
    return false;
  }
}

bool DVS_Driver::send_parameters() {
  uint8_t biases[12 * 3];

  uint32_t cas = parameters["cas"];
  biases[0] = (uint8_t) (cas >> 16);
  biases[1] = (uint8_t) (cas >> 8);
  biases[2] = (uint8_t) (cas >> 0);

  uint32_t injGnd = parameters["injGnd"];
  biases[3] = (uint8_t) (injGnd >> 16);
  biases[4] = (uint8_t) (injGnd >> 8);
  biases[5] = (uint8_t) (injGnd >> 0);

  uint32_t reqPd = parameters["reqPd"];
  biases[6] = (uint8_t) (reqPd >> 16);
  biases[7] = (uint8_t) (reqPd >> 8);
  biases[8] = (uint8_t) (reqPd >> 0);

  uint32_t puX = parameters["puX"];
  biases[9] = (uint8_t) (puX >> 16);
  biases[10] = (uint8_t) (puX >> 8);
  biases[11] = (uint8_t) (puX >> 0);

  uint32_t diffOff = parameters["diffOff"];
  biases[12] = (uint8_t) (diffOff >> 16);
  biases[13] = (uint8_t) (diffOff >> 8);
  biases[14] = (uint8_t) (diffOff >> 0);

  uint32_t req = parameters["req"];
  biases[15] = (uint8_t) (req >> 16);
  biases[16] = (uint8_t) (req >> 8);
  biases[17] = (uint8_t) (req >> 0);

  uint32_t refr = parameters["refr"];
  biases[18] = (uint8_t) (refr >> 16);
  biases[19] = (uint8_t) (refr >> 8);
  biases[20] = (uint8_t) (refr >> 0);

  uint32_t puY = parameters["puY"];
  biases[21] = (uint8_t) (puY >> 16);
  biases[22] = (uint8_t) (puY >> 8);
  biases[23] = (uint8_t) (puY >> 0);

  uint32_t diffOn = parameters["diffOn"];
  biases[24] = (uint8_t) (diffOn >> 16);
  biases[25] = (uint8_t) (diffOn >> 8);
  biases[26] = (uint8_t) (diffOn >> 0);

  uint32_t diff = parameters["diff"];
  biases[27] = (uint8_t) (diff >> 16);
  biases[28] = (uint8_t) (diff >> 8);
  biases[29] = (uint8_t) (diff >> 0);

  uint32_t foll = parameters["foll"];
  biases[30] = (uint8_t) (foll >> 16);
  biases[31] = (uint8_t) (foll >> 8);
  biases[32] = (uint8_t) (foll >> 0);

  uint32_t pr = parameters["pr"];
  biases[33] = (uint8_t) (pr >> 16);
  biases[34] = (uint8_t) (pr >> 8);
  biases[35] = (uint8_t) (pr >> 0);

  device_mutex.lock();
  libusb_control_transfer(device_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_SEND_BIASES, 0, 0, biases, sizeof(biases), 0);
  device_mutex.unlock();
}

} // namespace
