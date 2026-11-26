/*
 * FW10 USB CDC-ACM control endpoint (EP0) logic: descriptors, chapter-9 standard
 * requests, CDC class requests and EP0 IN chunking. Portable and host-tested; the
 * STM32 HAL PCD glue in firmware/target/target_usb_cdc.c only maps the returned
 * actions to HAL calls. Replaces the partial EP0 handler flagged as F-01/F-04.
 *
 * Full-speed device, bDeviceClass 0x02, one configuration:
 *   interface 0 CDC communication (ACM), notification EP 0x82 interrupt 16 B
 *   interface 1 CDC data, bulk OUT 0x01 / bulk IN 0x81, 64 B
 * Simulation/design only: enumeration on silicon is NOT verified.
 */

#ifndef WBR_USB_CDC_EP0_H
#define WBR_USB_CDC_EP0_H

#include <stdint.h>

#define USB_EP0_MPS                 64u
#define USB_CDC_DATA_OUT_EP         0x01u
#define USB_CDC_DATA_IN_EP          0x81u
#define USB_CDC_NOTIFY_EP           0x82u
#define USB_CDC_DATA_MPS            64u
#define USB_CDC_NOTIFY_MPS          16u

/* ST's development VID/PID for a virtual COM port; ASSUMED acceptable for an
 * internal bench device. Replace before any product use. */
#define USB_CDC_VID                 0x0483u
#define USB_CDC_PID                 0x5740u

#define USB_DEVICE_DESC_LEN         18u
#define USB_CONFIG_DESC_LEN         67u
#define USB_SERIAL_CHARS            24u

typedef enum {
    USB_EP0_ACT_STALL     = 0, /* stall EP0 IN and OUT */
    USB_EP0_ACT_STATUS_IN = 1, /* no data stage: send zero-length status IN */
    USB_EP0_ACT_DATA_IN   = 2, /* send data[0..len) then expect OUT status */
    USB_EP0_ACT_DATA_OUT  = 3  /* receive len bytes, then send status IN */
} UsbEp0ActKind;

typedef struct {
    uint8_t        kind;            /* UsbEp0ActKind */
    const uint8_t *data;            /* DATA_IN payload, already clipped to wLength */
    uint16_t       len;             /* DATA_IN / DATA_OUT length */
    uint8_t        zlp;             /* DATA_IN: needs a terminating zero-length packet */
    uint8_t        set_address;     /* apply `address` before the status stage */
    uint8_t        address;
    uint8_t        configure;       /* 1 = open data EPs (config 1), 2 = close them */
    uint8_t        clear_halt_ep;   /* endpoint address whose halt to clear, 0 = none */
    uint8_t        set_halt_ep;     /* endpoint address to halt, 0 = none */
} UsbEp0Action;

typedef struct {
    uint8_t  address;
    uint8_t  config_value;          /* 0 = addressed, 1 = configured */
    uint8_t  line_coding[7];
    uint8_t  ctrl_line_state;       /* bit0 DTR, bit1 RTS */
    uint8_t  pending_out_req;       /* class request waiting for its OUT data */
    uint8_t  reply[8];              /* GET_STATUS / GET_CONFIGURATION / GET_INTERFACE */
    uint8_t  serial_desc[2u + 2u * USB_SERIAL_CHARS];
    uint32_t setups;
    uint32_t stalls;
} UsbCdcEp0;

/* EP0 IN transfer chunker (data stage split into 64-byte packets + optional ZLP). */
typedef struct {
    const uint8_t *ptr;
    uint16_t       rem;
    uint8_t        zlp;
    uint8_t        active;
} UsbEp0InXfer;

typedef enum {
    USB_EP0_IN_SEND   = 0, /* transmit (*chunk, *chunk_len) on EP0 IN */
    USB_EP0_IN_STATUS = 1, /* data stage done: arm EP0 OUT for the zero-length status */
    USB_EP0_IN_IDLE   = 2  /* nothing in progress (e.g. completion of a status ZLP) */
} UsbEp0InStep;

void usb_cdc_ep0_init(UsbCdcEp0 *u, const uint32_t uid[3]);
void usb_cdc_ep0_bus_reset(UsbCdcEp0 *u);
void usb_cdc_ep0_setup(UsbCdcEp0 *u, const uint8_t setup[8], UsbEp0Action *act);
/* OUT data stage completed (SET_LINE_CODING). Returns 1 if it was expected. */
uint8_t usb_cdc_ep0_out_data(UsbCdcEp0 *u, const uint8_t *data, uint16_t len);

void usb_ep0_in_begin(UsbEp0InXfer *x, const uint8_t *data, uint16_t len, uint8_t zlp);
/* Call once to get the first packet, then once per EP0 IN transfer-complete. */
UsbEp0InStep usb_ep0_in_next(UsbEp0InXfer *x, const uint8_t **chunk, uint16_t *chunk_len);

/* Descriptor accessors for tests. */
const uint8_t *usb_cdc_device_descriptor(void);
const uint8_t *usb_cdc_config_descriptor(void);

#endif /* WBR_USB_CDC_EP0_H */
