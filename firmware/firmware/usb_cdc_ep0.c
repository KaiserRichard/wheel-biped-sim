/*
 * FW10 USB CDC-ACM EP0 logic. See usb_cdc_ep0.h.
 * Simulation/design only: host-tested request handling, not enumerated on silicon.
 */

#include "usb_cdc_ep0.h"

#include <stddef.h>
#include <string.h>

#define REQ_GET_STATUS          0x00u
#define REQ_CLEAR_FEATURE       0x01u
#define REQ_SET_FEATURE         0x03u
#define REQ_SET_ADDRESS         0x05u
#define REQ_GET_DESCRIPTOR      0x06u
#define REQ_GET_CONFIGURATION   0x08u
#define REQ_SET_CONFIGURATION   0x09u
#define REQ_GET_INTERFACE       0x0Au
#define REQ_SET_INTERFACE       0x0Bu

#define CDC_SET_LINE_CODING        0x20u
#define CDC_GET_LINE_CODING        0x21u
#define CDC_SET_CONTROL_LINE_STATE 0x22u
#define CDC_SEND_BREAK             0x23u

#define DESC_DEVICE             0x01u
#define DESC_CONFIGURATION      0x02u
#define DESC_STRING             0x03u

#define FEATURE_ENDPOINT_HALT   0x00u
#define FEATURE_REMOTE_WAKEUP   0x01u

static const uint8_t s_device_desc[USB_DEVICE_DESC_LEN] = {
    18u, DESC_DEVICE,
    0x00u, 0x02u,                   /* bcdUSB 2.00 */
    0x02u, 0x00u, 0x00u,            /* CDC device class (class at interface level) */
    USB_EP0_MPS,
    (uint8_t)(USB_CDC_VID & 0xFFu), (uint8_t)(USB_CDC_VID >> 8),
    (uint8_t)(USB_CDC_PID & 0xFFu), (uint8_t)(USB_CDC_PID >> 8),
    0x00u, 0x10u,                   /* bcdDevice 10.00 (FW10) */
    1u, 2u, 3u,                     /* iManufacturer, iProduct, iSerialNumber */
    1u                              /* bNumConfigurations */
};

static const uint8_t s_config_desc[USB_CONFIG_DESC_LEN] = {
    /* Configuration */
    9u, DESC_CONFIGURATION, USB_CONFIG_DESC_LEN, 0x00u, 2u, 1u, 0u,
    0xC0u,                          /* self-powered (CTRL board has its own supply) */
    50u,                            /* 100 mA */
    /* Interface 0: CDC communication, ACM, AT-command protocol */
    9u, 0x04u, 0u, 0u, 1u, 0x02u, 0x02u, 0x01u, 0u,
    /* CDC header functional descriptor, bcdCDC 1.10 */
    5u, 0x24u, 0x00u, 0x10u, 0x01u,
    /* Call management: no call management, data interface 1 */
    5u, 0x24u, 0x01u, 0x00u, 1u,
    /* ACM: SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE, SERIAL_STATE */
    4u, 0x24u, 0x02u, 0x02u,
    /* Union: master 0, slave 1 */
    5u, 0x24u, 0x06u, 0u, 1u,
    /* Notification endpoint */
    7u, 0x05u, USB_CDC_NOTIFY_EP, 0x03u, USB_CDC_NOTIFY_MPS, 0x00u, 0x10u,
    /* Interface 1: CDC data */
    9u, 0x04u, 1u, 0u, 2u, 0x0Au, 0x00u, 0x00u, 0u,
    /* Bulk OUT */
    7u, 0x05u, USB_CDC_DATA_OUT_EP, 0x02u, USB_CDC_DATA_MPS, 0x00u, 0x00u,
    /* Bulk IN */
    7u, 0x05u, USB_CDC_DATA_IN_EP, 0x02u, USB_CDC_DATA_MPS, 0x00u, 0x00u
};

static const uint8_t s_string_lang[4] = { 4u, DESC_STRING, 0x09u, 0x04u };
/* "WBR" */
static const uint8_t s_string_mfr[8] = { 8u, DESC_STRING, 'W', 0u, 'B', 0u, 'R', 0u };
/* "WBR CTRL rev4" */
static const uint8_t s_string_product[28] = {
    28u, DESC_STRING,
    'W', 0u, 'B', 0u, 'R', 0u, ' ', 0u, 'C', 0u, 'T', 0u, 'R', 0u, 'L', 0u,
    ' ', 0u, 'r', 0u, 'e', 0u, 'v', 0u, '4', 0u
};

const uint8_t *usb_cdc_device_descriptor(void)
{
    return s_device_desc;
}

const uint8_t *usb_cdc_config_descriptor(void)
{
    return s_config_desc;
}

static void set_stall(UsbCdcEp0 *u, UsbEp0Action *act)
{
    act->kind = (uint8_t)USB_EP0_ACT_STALL;
    u->stalls++;
}

static void data_in(UsbEp0Action *act, const uint8_t *data, uint16_t len, uint16_t w_length)
{
    const uint16_t n = (len < w_length) ? len : w_length;
    if (n == 0u) {
        act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN; /* wLength 0: no data stage */
        return;
    }
    act->kind = (uint8_t)USB_EP0_ACT_DATA_IN;
    act->data = data;
    act->len = n;
    /* A transfer shorter than requested that ends on a full packet needs a ZLP. */
    act->zlp = (n < w_length && n > 0u && (n % USB_EP0_MPS) == 0u) ? 1u : 0u;
}

static uint8_t ep_is_valid(uint8_t ep_addr)
{
    return (ep_addr == 0x00u || ep_addr == 0x80u || ep_addr == USB_CDC_DATA_OUT_EP
            || ep_addr == USB_CDC_DATA_IN_EP || ep_addr == USB_CDC_NOTIFY_EP) ? 1u : 0u;
}

void usb_cdc_ep0_bus_reset(UsbCdcEp0 *u)
{
    if (u == NULL) {
        return;
    }
    u->address = 0u;
    u->config_value = 0u;
    u->ctrl_line_state = 0u;
    u->pending_out_req = 0u;
}

void usb_cdc_ep0_init(UsbCdcEp0 *u, const uint32_t uid[3])
{
    static const char hex[] = "0123456789ABCDEF";
    if (u == NULL) {
        return;
    }
    memset(u, 0, sizeof(*u));
    /* 115200 8N1 (value is irrelevant for a USB virtual COM port). */
    u->line_coding[0] = 0x00u;
    u->line_coding[1] = 0xC2u;
    u->line_coding[2] = 0x01u;
    u->line_coding[3] = 0x00u;
    u->line_coding[4] = 0u;
    u->line_coding[5] = 0u;
    u->line_coding[6] = 8u;
    u->serial_desc[0] = (uint8_t)(2u + 2u * USB_SERIAL_CHARS);
    u->serial_desc[1] = DESC_STRING;
    for (uint32_t w = 0u; w < 3u; ++w) {
        const uint32_t word = (uid != NULL) ? uid[w] : 0u;
        for (uint32_t nib = 0u; nib < 8u; ++nib) {
            const uint32_t idx = w * 8u + nib;
            const uint32_t shift = 28u - 4u * nib;
            u->serial_desc[2u + 2u * idx] = (uint8_t)hex[(word >> shift) & 0xFu];
            u->serial_desc[3u + 2u * idx] = 0u;
        }
    }
    usb_cdc_ep0_bus_reset(u);
}

static void handle_get_descriptor(UsbCdcEp0 *u, uint16_t w_value, uint16_t w_length,
                                  UsbEp0Action *act)
{
    const uint8_t type = (uint8_t)(w_value >> 8);
    const uint8_t index = (uint8_t)(w_value & 0xFFu);
    switch (type) {
    case DESC_DEVICE:
        data_in(act, s_device_desc, USB_DEVICE_DESC_LEN, w_length);
        return;
    case DESC_CONFIGURATION:
        data_in(act, s_config_desc, USB_CONFIG_DESC_LEN, w_length);
        return;
    case DESC_STRING:
        if (index == 0u) {
            data_in(act, s_string_lang, (uint16_t)sizeof(s_string_lang), w_length);
            return;
        }
        if (index == 1u) {
            data_in(act, s_string_mfr, (uint16_t)sizeof(s_string_mfr), w_length);
            return;
        }
        if (index == 2u) {
            data_in(act, s_string_product, (uint16_t)sizeof(s_string_product), w_length);
            return;
        }
        if (index == 3u) {
            data_in(act, u->serial_desc, (uint16_t)sizeof(u->serial_desc), w_length);
            return;
        }
        break;
    default:
        /* Device qualifier (6) / other-speed (7): full-speed-only device must stall. */
        break;
    }
    set_stall(u, act);
}

static void handle_standard(UsbCdcEp0 *u, uint8_t bm, uint8_t req, uint16_t w_value,
                            uint16_t w_index, uint16_t w_length, UsbEp0Action *act)
{
    const uint8_t recipient = (uint8_t)(bm & 0x1Fu);
    const uint8_t dir_in = (uint8_t)(bm & 0x80u);
    switch (req) {
    case REQ_GET_STATUS:
        if (dir_in == 0u || w_length < 2u) {
            break;
        }
        u->reply[0] = (recipient == 0u) ? 0x01u : 0x00u; /* device: self-powered */
        u->reply[1] = 0u;
        if (recipient == 2u && ep_is_valid((uint8_t)(w_index & 0xFFu)) == 0u) {
            break;
        }
        data_in(act, u->reply, 2u, w_length);
        return;
    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
        if (recipient == 2u && w_value == FEATURE_ENDPOINT_HALT
            && ep_is_valid((uint8_t)(w_index & 0xFFu)) != 0u) {
            act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
            if ((w_index & 0x7Fu) != 0u) {
                if (req == REQ_CLEAR_FEATURE) {
                    act->clear_halt_ep = (uint8_t)(w_index & 0xFFu);
                } else {
                    act->set_halt_ep = (uint8_t)(w_index & 0xFFu);
                }
            }
            return;
        }
        if (recipient == 0u && w_value == FEATURE_REMOTE_WAKEUP) {
            act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN; /* accepted, never used */
            return;
        }
        break;
    case REQ_SET_ADDRESS:
        if (recipient == 0u && w_value <= 127u && w_length == 0u && u->config_value == 0u) {
            u->address = (uint8_t)w_value;
            act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
            act->set_address = 1u;
            act->address = (uint8_t)w_value;
            return;
        }
        break;
    case REQ_GET_DESCRIPTOR:
        if (dir_in != 0u) {
            handle_get_descriptor(u, w_value, w_length, act);
            return;
        }
        break;
    case REQ_GET_CONFIGURATION:
        if (dir_in != 0u && w_length >= 1u) {
            u->reply[0] = u->config_value;
            data_in(act, u->reply, 1u, w_length);
            return;
        }
        break;
    case REQ_SET_CONFIGURATION:
        if (w_value <= 1u) {
            act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
            act->configure = (w_value == 1u) ? 1u : 2u;
            u->config_value = (uint8_t)w_value;
            return;
        }
        break;
    case REQ_GET_INTERFACE:
        if (dir_in != 0u && u->config_value != 0u && w_index <= 1u && w_length >= 1u) {
            u->reply[0] = 0u;
            data_in(act, u->reply, 1u, w_length);
            return;
        }
        break;
    case REQ_SET_INTERFACE:
        if (u->config_value != 0u && w_index <= 1u && w_value == 0u) {
            act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
            return;
        }
        break;
    default:
        break;
    }
    set_stall(u, act);
}

static void handle_class(UsbCdcEp0 *u, uint8_t req, uint16_t w_value, uint16_t w_index,
                         uint16_t w_length, UsbEp0Action *act)
{
    if (w_index != 0u) { /* CDC requests address the communication interface */
        set_stall(u, act);
        return;
    }
    switch (req) {
    case CDC_SET_LINE_CODING:
        if (w_length == 7u) {
            act->kind = (uint8_t)USB_EP0_ACT_DATA_OUT;
            act->len = 7u;
            u->pending_out_req = CDC_SET_LINE_CODING;
            return;
        }
        break;
    case CDC_GET_LINE_CODING:
        if (w_length >= 1u) {
            data_in(act, u->line_coding, 7u, w_length);
            return;
        }
        break;
    case CDC_SET_CONTROL_LINE_STATE:
        u->ctrl_line_state = (uint8_t)(w_value & 0x03u);
        act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
        return;
    case CDC_SEND_BREAK:
        act->kind = (uint8_t)USB_EP0_ACT_STATUS_IN;
        return;
    default:
        break;
    }
    set_stall(u, act);
}

void usb_cdc_ep0_setup(UsbCdcEp0 *u, const uint8_t setup[8], UsbEp0Action *act)
{
    if (act == NULL) {
        return;
    }
    memset(act, 0, sizeof(*act));
    act->kind = (uint8_t)USB_EP0_ACT_STALL;
    if (u == NULL || setup == NULL) {
        return;
    }
    u->setups++;
    u->pending_out_req = 0u; /* a new SETUP cancels any unfinished control transfer */
    const uint8_t bm = setup[0];
    const uint8_t req = setup[1];
    const uint16_t w_value = (uint16_t)((uint16_t)setup[2] | ((uint16_t)setup[3] << 8));
    const uint16_t w_index = (uint16_t)((uint16_t)setup[4] | ((uint16_t)setup[5] << 8));
    const uint16_t w_length = (uint16_t)((uint16_t)setup[6] | ((uint16_t)setup[7] << 8));
    const uint8_t type = (uint8_t)((bm >> 5) & 0x03u);

    if (type == 0u) {
        handle_standard(u, bm, req, w_value, w_index, w_length, act);
    } else if (type == 1u && (bm & 0x1Fu) == 1u) {
        handle_class(u, req, w_value, w_index, w_length, act);
    } else {
        set_stall(u, act);
    }
}

uint8_t usb_cdc_ep0_out_data(UsbCdcEp0 *u, const uint8_t *data, uint16_t len)
{
    if (u == NULL || data == NULL || u->pending_out_req != CDC_SET_LINE_CODING || len != 7u) {
        if (u != NULL) {
            u->pending_out_req = 0u;
        }
        return 0u;
    }
    memcpy(u->line_coding, data, 7u);
    u->pending_out_req = 0u;
    return 1u;
}

void usb_ep0_in_begin(UsbEp0InXfer *x, const uint8_t *data, uint16_t len, uint8_t zlp)
{
    if (x == NULL) {
        return;
    }
    x->ptr = data;
    x->rem = len;
    x->zlp = zlp;
    x->active = 1u;
}

UsbEp0InStep usb_ep0_in_next(UsbEp0InXfer *x, const uint8_t **chunk, uint16_t *chunk_len)
{
    if (chunk != NULL) {
        *chunk = NULL;
    }
    if (chunk_len != NULL) {
        *chunk_len = 0u;
    }
    if (x == NULL || x->active == 0u) {
        return USB_EP0_IN_IDLE;
    }
    if (x->rem > 0u) {
        const uint16_t n = (x->rem > USB_EP0_MPS) ? (uint16_t)USB_EP0_MPS : x->rem;
        if (chunk != NULL) {
            *chunk = x->ptr;
        }
        if (chunk_len != NULL) {
            *chunk_len = n;
        }
        x->ptr += n;
        x->rem = (uint16_t)(x->rem - n);
        return USB_EP0_IN_SEND;
    }
    if (x->zlp != 0u) {
        x->zlp = 0u;
        return USB_EP0_IN_SEND; /* zero-length packet */
    }
    x->active = 0u;
    return USB_EP0_IN_STATUS;
}
