#if defined(RP2040)
extern "C"
{
  #include <string.h>
  #include <hardware/resets.h>
  #include <hardware/irq.h>
  #include <hardware/regs/usb.h>
  #include <hardware/structs/usb.h>
}

#include "usb_common.h"
#include "usb_const.h"

using usb_ep_handler = void (*)([[maybe_unused]] unsigned char *buf,[[maybe_unused]] uint16_t len);

// in which we keep the endpoint configuration
struct usb_endpoint_configuration {
    const  usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    // Pointers to endpoint + buffer control registers
    // in the USB controller DPSRAM
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    // Toggle after each packet (unless replying to a SETUP)
    uint8_t next_pid;
};

// in which we keep the device configuration
struct usb_device_configuration {
    const usb_device_descriptor *device_descriptor;
    const usb_interface_descriptor *interface_descriptor;
    const usb_configuration_descriptor *config_descriptor;
    const unsigned char *lang_descriptor;
    const char **descriptor_strings;
    // USB num endpoints is 16
    usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};

#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_IN_ADDR  (USB_DIR_IN  | 2)

// EP0 IN and OUT
static const usb_endpoint_descriptor ep0_out = {
    .bLength          = sizeof(usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR, // EP number 0, OUT from host (rx to device)
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const usb_endpoint_descriptor ep0_in = {
    .bLength          = sizeof(usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_IN_ADDR, // EP number 0, OUT from host (rx to device)
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

// Descriptors
static const usb_device_descriptor device_descriptor = {
    .bLength         = sizeof(usb_device_descriptor),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB          = 0x0110, // USB 1.1 device
    .bDeviceClass    = 0,      // Specified in interface descriptor
    .bDeviceSubClass = 0,      // No subclass
    .bDeviceProtocol = 0,      // No protocol
    .bMaxPacketSize0 = 64,     // Max packet size for ep0
    .idVendor        = 0x0000, // Your vendor id
    .idProduct       = 0x0001, // Your product ID
    .bcdDevice       = 0,      // No device revision number
    .iManufacturer   = 1,      // Manufacturer string index
    .iProduct        = 2,      // Product string index
    .iSerialNumber = 0,        // No serial number
    .bNumConfigurations = 1    // One configuration
};

static const usb_interface_descriptor interface_descriptor = {
    .bLength            = sizeof(usb_interface_descriptor),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,    // Interface has 2 endpoints
    .bInterfaceClass    = 0xff, // Vendor specific endpoint
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface         = 0
};

static const usb_endpoint_descriptor ep1_out = {
    .bLength          = sizeof(usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP1_OUT_ADDR, // EP number 1, OUT from host (rx to device)
    .bmAttributes     = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const usb_endpoint_descriptor ep2_in = {
    .bLength          = sizeof(usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP2_IN_ADDR, // EP number 2, IN from host (tx from device)
    .bmAttributes     = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const usb_configuration_descriptor config_descriptor = {
    .bLength         = sizeof(usb_configuration_descriptor),
    .bDescriptorType = USB_DT_CONFIG,
    .wTotalLength    = (sizeof(config_descriptor) +
                        sizeof(interface_descriptor) +
                        sizeof(ep1_out) +
                        sizeof(ep2_in)),
    .bNumInterfaces  = 1,
    .bConfigurationValue = 1, // Configuration 1
    .iConfiguration = 0,      // No string
    .bmAttributes = 0xc0,     // attributes: self powered, no remote wakeup
    .bMaxPower = 0x32         // 100ma
};

static const unsigned char lang_descriptor[] = {
    4,         // bLength
    0x03,      // bDescriptorType == String Descriptor
    0x09, 0x04 // language id = us english
};

static const char *descriptor_strings[] = {
    "Pico Weather Station",    // Vendor
    "Pico Weather Station Receiver" // Product
};

void usb_device_init();


// <cmd 1 B> <data len 1 B> <data 0 B up to 62 B>
// cmd: CCCC RREE
// C -> command bit
// R -> reserve bit (0)
// E -> error response bit 

struct UsbData
{
    uint8_t hasData = 0;
    uint8_t data[1024]{0};
    uint16_t dataLen = 0;
};

extern UsbData usbData;

constexpr UsbCommand getUsbCommand(uint8_t data)
{
    return static_cast<UsbCommand>(data & usbCommandMask);
}

constexpr uint8_t getUsbResponseCommand(UsbCommand usbCommand, UsbErrorCode usbErrorCode)
{
    return static_cast<uint8_t>(usbCommand) | static_cast<uint8_t>(usbErrorCode);
}
#endif