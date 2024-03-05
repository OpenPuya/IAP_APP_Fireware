#include "usbd_core.h"
#include "usbd_hid.h"
#include "app_wdg.h"
#include "app_bootloader.h"

/*!< hidraw in endpoint */
#define HIDRAW_IN_EP       0x81
#define HIDRAW_IN_SIZE     64
#define HIDRAW_IN_INTERVAL 1

/*!< hidraw out endpoint */
#define HIDRAW_OUT_EP          0x02
#define HIDRAW_OUT_EP_SIZE     64
#define HIDRAW_OUT_EP_INTERVAL 1

/*Puya 0x50 0x75 0x79 0x61*/
#define USBD_VID           0xFFFF
//#define USBD_PID           0xFFFF
#define USBD_PID           PID
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_HID_CONFIG_DESC_SIZ (9 + 9 + 9 + 7 + 7)

/*!< custom hid report descriptor size */
#define HID_CUSTOM_REPORT_DESC_SIZE 34

/*!< global descriptor */
static const uint8_t hid_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0002, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_HID_CONFIG_DESC_SIZ, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /************** Descriptor of Custom interface *****************/
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
    0x00,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x02,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,                             /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID ********************/
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,                        /* bCountryCode: Hardware target country */
    0x01,                        /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,                        /* bDescriptorType */
    HID_CUSTOM_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Custom in endpoint ********************/
    0x07,                         /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
    HIDRAW_IN_EP,                 /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                         /* bmAttributes: Interrupt endpoint */
    WBVAL(HIDRAW_IN_SIZE),        /* wMaxPacketSize: 4 Byte max */
    HIDRAW_IN_INTERVAL,           /* bInterval: Polling Interval */
    /******************** Descriptor of Custom out endpoint ********************/
    0x07,                         /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
    HIDRAW_OUT_EP,                /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                         /* bmAttributes: Interrupt endpoint */
    WBVAL(HIDRAW_OUT_EP_SIZE),    /* wMaxPacketSize: 4 Byte max */
    HIDRAW_OUT_EP_INTERVAL,       /* bInterval: Polling Interval */
    /* 73 */
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x08,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'R', 0x00,                  /* wcChar1 */
    'M', 0x00,                  /* wcChar2 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x2E,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'R', 0x00,                  /* wcChar1 */
    'M', 0x00,                  /* wcChar2 */
    ' ', 0x00,                  /* wcChar3 */
    'D', 0x00,                  /* wcChar4 */
    'e', 0x00,                  /* wcChar5 */
    'v', 0x00,                  /* wcChar6 */
    'i', 0x00,                  /* wcChar7 */
    'c', 0x00,                  /* wcChar8 */
    'e', 0x00,                  /* wcChar9 */
    ' ', 0x00,                  /* wcChar10 */
    'i', 0x00,                  /* wcChar11 */
    'n', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'F', 0x00,                  /* wcChar15 */
    'U', 0x00,                  /* wcChar16 */
    ' ', 0x00,                  /* wcChar17 */
    'M', 0x00,                  /* wcChar18 */
    'o', 0x00,                  /* wcChar19 */
    'd', 0x00,                  /* wcChar20 */
    'e', 0x00,                  /* wcChar21 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '3', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '2', 0x00,                  /* wcChar6 */
    '3', 0x00,                  /* wcChar7 */
    '1', 0x00,                  /* wcChar8 */
    '0', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

/*!< custom hid report descriptor */
static const uint8_t hid_custom_report_desc[HID_CUSTOM_REPORT_DESC_SIZE] = {
    /* USER CODE BEGIN 0 */
    0x06, 0x00, 0xff, // USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,       // USAGE (Vendor Usage 1)
    0xa1, 0x01,       // COLLECTION (Application)
    0x09, 0x01,       //   USAGE (Vendor Usage 1)
    0x15, 0x00,       //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00, //   LOGICAL_MAXIMUM (255)
    0x95, 0x40,       //   REPORT_COUNT (64)
    0x75, 0x08,       //   REPORT_SIZE (8)
    0x81, 0x02,       //   INPUT (Data,Var,Abs)
    /* <___________________________________________________> */
    0x09, 0x01,       //   USAGE (Vendor Usage 1)
    0x15, 0x00,       //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00, //   LOGICAL_MAXIMUM (255)
    0x95, 0x40,       //   REPORT_COUNT (64)
    0x75, 0x08,       //   REPORT_SIZE (8)
    0x91, 0x02,       //   OUTPUT (Data,Var,Abs)
    /* USER CODE END 0 */
    0xC0 /*     END_COLLECTION               */
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t guc_usbd_read_buffer[64];

__IO uint8_t guc_usbd_hid_custom_out_callback_flag = 0;

#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1

/*!< hid state ! Data can be sent only when state is idle  */
static volatile uint8_t custom_state = HID_STATE_IDLE;

void usbd_configure_done_callback(void)
{
    /* setup first out ep read transfer */
    usbd_ep_start_read(HIDRAW_OUT_EP, guc_usbd_read_buffer, 64);
}

static void usbd_hid_custom_in_callback(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);
    custom_state = HID_STATE_IDLE;
}

static void usbd_hid_custom_out_callback(uint8_t ep, uint32_t nbytes)
{
    guc_usbd_hid_custom_out_callback_flag = 1;
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    usbd_ep_start_read(HIDRAW_OUT_EP, guc_usbd_read_buffer, 64);
}

static struct usbd_endpoint custom_in_ep = {
    .ep_cb = usbd_hid_custom_in_callback,
    .ep_addr = HIDRAW_IN_EP
};

static struct usbd_endpoint custom_out_ep = {
    .ep_cb = usbd_hid_custom_out_callback,
    .ep_addr = HIDRAW_OUT_EP
};

struct usbd_interface intf0;

void hid_custom_dfu_init(void)
{
    usbd_desc_register(hid_descriptor);
    usbd_add_interface(usbd_hid_init_intf(&intf0, hid_custom_report_desc, HID_CUSTOM_REPORT_DESC_SIZE));
    usbd_add_endpoint(&custom_in_ep);
    usbd_add_endpoint(&custom_out_ep);

    usbd_initialize();
}

int hid_custom_dfu_shake_hands_check(void)
{
    if (0 == guc_usbd_hid_custom_out_callback_flag)
    {
      return ERROR;
    }
    
    guc_usbd_hid_custom_out_callback_flag = 0;
    
    for (int i=0; i<64; i++)
    {
      if (0x7F != guc_usbd_read_buffer[i])
      {
        return ERROR;
      }
    }
    
    return SUCCESS;
}

int hid_custom_dfu_send_data(uint8_t *data, uint16_t size)
{
    uint32_t i=0;
  
    uint8_t sendbuffer[64];
    
    for (i=0; i<size/64; i++)
    {

      int ret = usbd_ep_start_write(HIDRAW_IN_EP, data + i*64, 64);
      if (ret < 0) {
          return ERROR;
      }
      custom_state = HID_STATE_BUSY;
      while (custom_state == HID_STATE_BUSY)
      {
        APP_WDG_Refresh();
      }
    }
    
    if (size % 64)
    {
      memset(sendbuffer, 0x00, 64);
      memcpy(sendbuffer, data + i*64, size % 64);

      int ret = usbd_ep_start_write(HIDRAW_IN_EP, sendbuffer, 64);
      if (ret < 0) {
          return ERROR;
      }
      custom_state = HID_STATE_BUSY;
      while (custom_state == HID_STATE_BUSY)
      {
        APP_WDG_Refresh();
      }
    }
      
    return SUCCESS;
}

void hid_custom_dfu_read_data(uint8_t *data, uint16_t size)
{
    uint32_t i=0;  
    
    for (i=0; i<size/64; i++)
    {
      while (0 == guc_usbd_hid_custom_out_callback_flag)
      {
        APP_WDG_Refresh();
      }
      guc_usbd_hid_custom_out_callback_flag = 0;
      memcpy(data + i*64, guc_usbd_read_buffer, 64);
    }
    
    if (size % 64)
    {
      while (0 == guc_usbd_hid_custom_out_callback_flag)
      {
        APP_WDG_Refresh();
      }
      guc_usbd_hid_custom_out_callback_flag = 0;
      memcpy(data + i*64, guc_usbd_read_buffer, size % 64);
    }
}