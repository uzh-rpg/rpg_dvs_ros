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

#include "dvs_driver/dvs_driver.h"

namespace dvs {

extern "C" { 

static void callback_wrapper(struct libusb_transfer *transfer) 
{ 
  DvsDriver *dvs_driver = (DvsDriver *)transfer->user_data;

  dvs_driver->callback(transfer);
} 

} // extern "C"

DvsDriver::DvsDriver(std::string dvs_serial_number, bool master)
{
  // initialize parameters (min, max, value)
  parameters_.insert(std::pair<std::string, Parameter>("cas", Parameter(0, 16777215, 1992)));
  parameters_.insert(std::pair<std::string, Parameter>("injGnd", Parameter(0, 16777215, 1108364)));
  parameters_.insert(std::pair<std::string, Parameter>("reqPd", Parameter(0, 16777215, 16777215)));
  parameters_.insert(std::pair<std::string, Parameter>("puX", Parameter(0, 16777215, 8159221)));
  parameters_.insert(std::pair<std::string, Parameter>("diffOff", Parameter(0, 16777215, 132)));
  parameters_.insert(std::pair<std::string, Parameter>("req", Parameter(0, 16777215, 309590)));
  parameters_.insert(std::pair<std::string, Parameter>("refr", Parameter(0, 16777215, 969)));
  parameters_.insert(std::pair<std::string, Parameter>("puY", Parameter(0, 16777215, 16777215)));
  parameters_.insert(std::pair<std::string, Parameter>("diffOn", Parameter(0, 16777215, 209996)));
  parameters_.insert(std::pair<std::string, Parameter>("diff", Parameter(0, 16777215, 13125)));
  parameters_.insert(std::pair<std::string, Parameter>("foll", Parameter(0, 16777215, 271)));
  parameters_.insert(std::pair<std::string, Parameter>("Pr", Parameter(0, 16777215, 217)));

  wrap_add_ = 0;
  last_timestamp_ = 0;

  libusb_init(NULL);

  device_mutex_.lock();
  device_running_ = openDevice(dvs_serial_number);
  device_mutex_.unlock();

  if (device_running_)
  {
    std::cout << "Opened DVS with the following identification: " << camera_id_ << std::endl;
  }
  else
  {
    if (dvs_serial_number.empty())
    {
      std::cout << "Could not find any DVS..." << std::endl;
    }
    else
    {
      std::cout << "Could not find DVS with serial number " << dvs_serial_number << "..." << std::endl;
    }
  }


  // put into slave mode?
  if (!master)
  {
    std::cout << "Setting camera (" << camera_id_ << ") as slave!" << std::endl;
    device_mutex_.lock();

    unsigned char enabled[1];
    enabled[0] = 0;

    libusb_control_transfer(device_handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                            VENDOR_REQUEST_SET_SYNC_ENABLED, 0, 0, enabled, sizeof(enabled), 0);
    device_mutex_.unlock();
  }

  if (device_running_)
    thread_ = new boost::thread(boost::bind(&DvsDriver::run, this));
}

DvsDriver::~DvsDriver()
{
  device_mutex_.lock();
  closeDevice();
  device_mutex_.unlock();
}

bool DvsDriver::openDevice(std::string dvs_serial_number)
{
  // get device list
  libusb_device **list = NULL;
  ssize_t count = libusb_get_device_list(NULL, &list);

  // iterate over all USB devices
  for (size_t idx = 0; idx < count; ++idx)
  {
      libusb_device *device = list[idx];
      libusb_device_descriptor desc = {0};

      int rc = libusb_get_device_descriptor(device, &desc);
      assert(rc == 0);

//      printf("Vendor:Device = %04x:%04x\n", desc.idVendor, desc.idProduct);

      if (DVS128_VID == desc.idVendor && DVS128_PID == desc.idProduct)
      {

        // Open device to see its serial number
        int e = libusb_open(device, &device_handle_);
        if (e)
        {
          printf("Error opening device... Error code: %d", e);
          continue;
        }

        unsigned char sSerial[256];

        e = libusb_get_string_descriptor_ascii(device_handle_, desc.iSerialNumber, sSerial, sizeof(sSerial));
        if (e < 0)
        {
          std::cout << "libusb: get error code: " << e << std::endl;
          libusb_close(device_handle_);
        }

        if (std::string(reinterpret_cast<char*>(sSerial)) == dvs_serial_number || dvs_serial_number.empty())
        {
          // claim interface
          libusb_claim_interface(device_handle_, 0);

          // figure out precise product
          unsigned char sProduct[256];
          e = libusb_get_string_descriptor_ascii(device_handle_, desc.iProduct, sProduct, sizeof(sProduct));
          if (e < 0)
          {
            std::cout << "libusb: get error code: " << e << std::endl;
            libusb_close(device_handle_);
          }

          camera_id_ = std::string(reinterpret_cast<char*>(sProduct)) + "-" + std::string(reinterpret_cast<char*>(sSerial));

          // alloc transfer and setup
          transfer_ = libusb_alloc_transfer(0);

          transfer_->length = (int) bufferSize;
          buffer_ = new unsigned char[bufferSize];
          transfer_->buffer = buffer_;

          transfer_->dev_handle = device_handle_;
          transfer_->endpoint = USB_IO_ENDPOINT;
          transfer_->type = LIBUSB_TRANSFER_TYPE_BULK;
          transfer_->callback = callback_wrapper;
          transfer_->user_data = this;
          transfer_->timeout = 0;
          transfer_->flags = LIBUSB_TRANSFER_FREE_BUFFER;

          //  libusb_submit_transfer(transfer);
          int status = libusb_submit_transfer(transfer_);
          if (status != LIBUSB_SUCCESS)
          {
            std::cout << "Unable to submit libusb transfer: " << status << std::endl;

            // The transfer buffer is freed automatically here thanks to
            // the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
            libusb_free_transfer(transfer_);
          }

          return true;
        }

        libusb_close(device_handle_);
      }
  }
  return false;
}

void DvsDriver::closeDevice()
{
  // stop thread
  thread_->join();

  // close transfer
  int status = libusb_cancel_transfer(transfer_);
  if (status != LIBUSB_SUCCESS && status != LIBUSB_ERROR_NOT_FOUND) {
    std::cout << "Unable to cancel libusb transfer: " << status << std::endl;
  }

  libusb_close(device_handle_);
  libusb_exit(NULL);
}

void DvsDriver::run()
{
  timeval te;
  te.tv_sec = 0;
  te.tv_usec = 1000000;

  libusb_control_transfer(device_handle_,
                          LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0, 0);

  try
  {
    int completed;
    while (true)
    {
      device_mutex_.lock();
      libusb_handle_events_timeout(NULL, &te);
      device_mutex_.unlock();
    }
  }
  catch (boost::thread_interrupted const& )
  {
    //clean resources
    std::cout << "Worker thread interrupted!" << std::endl;
  }

  libusb_control_transfer(device_handle_,
                          LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_STOP_TRANSFER, 0, 0, NULL, 0, 0);
}

void DvsDriver::callback(struct libusb_transfer *transfer)
{
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    // Handle data.
    event_buffer_mutex_.lock();
    eventTranslator(transfer->buffer, (size_t) transfer->actual_length);
    event_buffer_mutex_.unlock();
  }

  if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE)
  {
    // Submit transfer again.
    if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS)
    {
      return;
    }
  }

  // Cannot recover (cancelled, no device, or other critical error).
  // Signal this by adjusting the counter, free and exit.
  libusb_free_transfer(transfer);
}

void DvsDriver::eventTranslator(uint8_t *buffer, size_t bytesSent)
{
  // Truncate off any extra partial event.
  bytesSent &= (size_t) ~0x03;

  for (size_t i = 0; i < bytesSent; i += 4)
  {
    if ((buffer[i + 3] & 0x80) == 0x80)
    {
      // timestamp bit 15 is one -> wrap: now we need to increment
      // the wrapAdd, uses only 14 bit timestamps
      wrap_add_ += 0x4000;

      // Detect big timestamp wrap-around.
      if (wrap_add_ == 0)
      {
        // Reset lastTimestamp to zero at this point, so we can again
        // start detecting overruns of the 32bit value.
        last_timestamp_ = 0;
      }
    }
    else if ((buffer[i + 3] & 0x40) == 0x40)
    {
      // timestamp bit 14 is one -> wrapAdd reset: this firmware
      // version uses reset events to reset timestamps
      wrap_add_ = 0;
      last_timestamp_ = 0;
    }
    else
    {
      // address is LSB MSB (USB is LE)
      uint16_t addressUSB = le16toh(*((uint16_t * ) (&buffer[i])));

      // same for timestamp, LSB MSB (USB is LE)
      // 15 bit value of timestamp in 1 us tick
      uint16_t timestampUSB = le16toh(*((uint16_t * ) (&buffer[i + 2])));

      // Expand to 32 bits. (Tick is 1µs already.)
      uint64_t timestamp = timestampUSB + wrap_add_;

      // Check monotonicity of timestamps.
      if (timestamp < last_timestamp_)
      {
         std::cout << "DVS128: non-monotonic time-stamp detected: lastTimestamp=" << last_timestamp_ << ", timestamp=" << timestamp << "." << std::endl;
      }

      last_timestamp_ = timestamp;

      // Special Trigger Event (MSB is set)
      if ((addressUSB & DVS128_SYNC_EVENT_MASK) != 0)
      {
        std::cout << "got sync event on camera " << camera_id_ << "at " << timestamp << std::endl;
      }
      else
      {
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
        event_buffer_.push_back(e);
      }
    }
  }
}

std::vector<Event> DvsDriver::getEvents()
{
  event_buffer_mutex_.lock();
  std::vector<Event> buffer_copy = event_buffer_;
  event_buffer_.clear();
  event_buffer_mutex_.unlock();
  return buffer_copy;
}

void DvsDriver::resetTimestamps()
{
  device_mutex_.lock();
  libusb_control_transfer(device_handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_RESET_TIMESTAMPS, 0, 0, NULL, 0, 0);
  device_mutex_.unlock();
}

bool DvsDriver::changeParameter(std::string parameter, uint32_t value)
{
  // does parameter exist?
  if (parameters_.find(parameter) != parameters_.end())
  {
    // did it change? (only if within range)
    return (parameters_[parameter].setValue(value));
  }
  else
  {
    return false;
  }
}

bool DvsDriver::changeParameters(uint32_t cas, uint32_t injGnd, uint32_t reqPd, uint32_t puX,
                                   uint32_t diffOff, uint32_t req, uint32_t refr, uint32_t puY,
                                   uint32_t diffOn, uint32_t diff, uint32_t foll, uint32_t Pr)
{
  changeParameter("cas", cas);
  changeParameter("injGnd", injGnd);
  changeParameter("reqPd", reqPd);
  changeParameter("puX", puX);
  changeParameter("diffOff", diffOff);
  changeParameter("req", req);
  changeParameter("refr", refr);
  changeParameter("puY", puY);
  changeParameter("diffOn", diffOn);
  changeParameter("diff", diff);
  changeParameter("foll", foll);
  changeParameter("Pr", Pr);

  return sendParameters();
}

bool DvsDriver::sendParameters()
{
  uint8_t biases[12 * 3];

  uint32_t cas = parameters_["cas"].getValue();
  biases[0] = (uint8_t) (cas >> 16);
  biases[1] = (uint8_t) (cas >> 8);
  biases[2] = (uint8_t) (cas >> 0);

  uint32_t injGnd = parameters_["injGnd"].getValue();
  biases[3] = (uint8_t) (injGnd >> 16);
  biases[4] = (uint8_t) (injGnd >> 8);
  biases[5] = (uint8_t) (injGnd >> 0);

  uint32_t reqPd = parameters_["reqPd"].getValue();
  biases[6] = (uint8_t) (reqPd >> 16);
  biases[7] = (uint8_t) (reqPd >> 8);
  biases[8] = (uint8_t) (reqPd >> 0);

  uint32_t puX = parameters_["puX"].getValue();
  biases[9] = (uint8_t) (puX >> 16);
  biases[10] = (uint8_t) (puX >> 8);
  biases[11] = (uint8_t) (puX >> 0);

  uint32_t diffOff = parameters_["diffOff"].getValue();
  biases[12] = (uint8_t) (diffOff >> 16);
  biases[13] = (uint8_t) (diffOff >> 8);
  biases[14] = (uint8_t) (diffOff >> 0);

  uint32_t req = parameters_["req"].getValue();
  biases[15] = (uint8_t) (req >> 16);
  biases[16] = (uint8_t) (req >> 8);
  biases[17] = (uint8_t) (req >> 0);

  uint32_t refr = parameters_["refr"].getValue();
  biases[18] = (uint8_t) (refr >> 16);
  biases[19] = (uint8_t) (refr >> 8);
  biases[20] = (uint8_t) (refr >> 0);

  uint32_t puY = parameters_["puY"].getValue();
  biases[21] = (uint8_t) (puY >> 16);
  biases[22] = (uint8_t) (puY >> 8);
  biases[23] = (uint8_t) (puY >> 0);

  uint32_t diffOn = parameters_["diffOn"].getValue();
  biases[24] = (uint8_t) (diffOn >> 16);
  biases[25] = (uint8_t) (diffOn >> 8);
  biases[26] = (uint8_t) (diffOn >> 0);

  uint32_t diff = parameters_["diff"].getValue();
  biases[27] = (uint8_t) (diff >> 16);
  biases[28] = (uint8_t) (diff >> 8);
  biases[29] = (uint8_t) (diff >> 0);

  uint32_t foll = parameters_["foll"].getValue();
  biases[30] = (uint8_t) (foll >> 16);
  biases[31] = (uint8_t) (foll >> 8);
  biases[32] = (uint8_t) (foll >> 0);

  uint32_t Pr = parameters_["Pr"].getValue();
  biases[33] = (uint8_t) (Pr >> 16);
  biases[34] = (uint8_t) (Pr >> 8);
  biases[35] = (uint8_t) (Pr >> 0);

  device_mutex_.lock();
  libusb_control_transfer(device_handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                          VENDOR_REQUEST_SEND_BIASES, 0, 0, biases, sizeof(biases), 0);
  device_mutex_.unlock();
}

} // namespace
