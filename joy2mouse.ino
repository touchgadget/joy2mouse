/*********************************************************************
MIT License

Copyright (c) 2023 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* Demostrate converting Logitech flight joystick to mouse.
 *
 * Some code is taken from an Adafruit example program so Adafruit's
 * copyright is included.
 *
 * - Device run on the native usb controller with type C USB connector.
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library
 *   with type A USB connector.
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set this to 0 for best performance.
#define USB_DEBUG 0

#if USB_DEBUG
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

// Logitech Extreme 3D Pro flight joystick HID report layout
// Large joystick X, Y, Z (twist) axes
// 8 way hat switch
// 12 buttons (6 on the stick, 6 on the base)
// throttle slider
typedef struct __attribute__ ((packed)) {
  uint32_t x : 10;      // 0..512..1023
  uint32_t y : 10;      // 0..512..1023
  uint32_t hat : 4;
  uint32_t twist : 8;   // 0..127..255
  uint8_t buttons_a;
  uint8_t slider;       // 0..255
  uint8_t buttons_b;
} Logitech_E3DP_t ;

typedef struct {
  const uint16_t  DEADZONE;
  const uint16_t  MIN;
  const uint16_t  CENTER;
  const uint16_t  MAX;
} Axis_Control_t;

// Logitech Extreme 3D Pro flight joystick state
typedef struct {
  Logitech_E3DP_t report;
  const uint16_t USB_VID = 0x046d;
  const uint16_t USB_PID = 0xc215;
  const Axis_Control_t X_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Y_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Twist_Control = { 4, 0, 127, 255 };
  const Axis_Control_t Speed_Control = { 4, 0, 127, 255 };
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
} Logitech_E3DP_state_t;

volatile Logitech_E3DP_state_t Le3dp;

// Experiment with mouse acceleration/sensitivity. For small deflections of the
// joystick move the mouse slowly. For larger deflections, move the mouse
// faster. The look up table is computed outside the main loop to avoid calling
// time consuming the math function on every loop. The X and Y tables use the
// same values but could use different values.
const int DEADZONE = 60;
const float X_SPEED = 3.0f;   // sensitivity
const float Y_SPEED = 3.0f;   // sensitivity
int8_t X_ACCEL[(1024/2) + 1];
int8_t Y_ACCEL[(1024/2) + 1];

// pio-usb is required for rp2040 usb host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"

// USB Host object for Logitech joystick
Adafruit_USBH_Host USBHost;

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);

void init_accel() {
  for (size_t i = 0; i < sizeof(X_ACCEL); i++ ) {
    if (i < DEADZONE) {
      // Dead zone around center to fix drift.
      // Joysticks do not alway report (x=0,y=0) when the stick is released.
      // When this happens the cursor moves slowly when it should not be moving.
      // This is sometimes called drifting. Drifting is fixed by treating
      // joystick values between -DEADZONE to +DEADZONE as 0. If the cursor is
      // still drifting increase the value of DEADZONE.
      X_ACCEL[i] = 0;
      Y_ACCEL[i] = 0;
    } else {
      X_ACCEL[i] = max(1, int((pow(i/511.0f, X_SPEED) * 127) + 0.5f));
      Y_ACCEL[i] = max(1, int((pow(i/511.0f, Y_SPEED) * 127) + 0.5f));
    }
  }
  X_ACCEL[512] = X_ACCEL[511];
  Y_ACCEL[512] = Y_ACCEL[511];
}

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
#if USB_DEBUG
  Serial.begin(115200);
#else
  Serial.end();
#endif
  usb_hid.begin();
  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif

  init_accel();
  DBG_println("Joystick to mouse");
}

void loop() {
  if (Le3dp.connected) {
    if (Le3dp.available) {
      if (sizeof(Le3dp.report) == Le3dp.report_len) {
#if USB_DEBUG
        // Hex dump the HID report
        uint8_t *rpt = (uint8_t *)&Le3dp.report;
        DBG_printf("LE3DP report(%d): ", Le3dp.report_len);
        for (uint16_t i = 0; i < Le3dp.report_len; i++) {
          DBG_printf("0x%02X ", rpt[i]);
        }
        DBG_println();
        DBG_printf("X:%d,Y:%d,hat:%d,twist:%d,slider:%d,"\
            "buttons_a:0x%x,buttons_b:0x%x\r\n",
            Le3dp.report.x, Le3dp.report.y, Le3dp.report.hat, Le3dp.report.twist,
            Le3dp.report.slider, Le3dp.report.buttons_a, Le3dp.report.buttons_b);
#endif
      }
      if (usb_hid.ready()) {
        hid_mouse_report_t mouseRpt = {0};
        mouseRpt.buttons = Le3dp.report.buttons_a;
        int signed_x = Le3dp.report.x - 511;
        int signed_y = Le3dp.report.y - 511;
        if (signed_x < 0) {
          mouseRpt.x = -X_ACCEL[-signed_x];
        } else {
          mouseRpt.x = X_ACCEL[signed_x];
        }
        if (signed_y < 0) {
          mouseRpt.y = -Y_ACCEL[-signed_y];
        } else {
          mouseRpt.y = Y_ACCEL[signed_y];
        }
        DBG_printf("X:%d,%d, Y:%d,%d\r\n", signed_x, mouseRpt.x, signed_y, mouseRpt.y);
        usb_hid.sendReport(0, (void *)&mouseRpt, sizeof(mouseRpt));
      }

      Le3dp.available = false;
    }
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
#if USB_DEBUG
    while (!Serial) { delay(1); }
#endif
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while(1) delay(1);
  }

#ifdef PIN_PIO_USB_HOST_VBUSEN
  pinMode(PIN_PIO_USB_HOST_VBUSEN, OUTPUT);
  digitalWrite(PIN_PIO_USB_HOST_VBUSEN, PIN_PIO_USB_HOST_VBUSEN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1()
{
  USBHost.task();
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  if ((vid == Le3dp.USB_VID) && (pid == Le3dp.USB_PID)) {
    DBG_printf("Logitech Extreme 3D Pro (%d) connected\r\n", instance);
    Le3dp.connected = true;
    Le3dp.available = false;
    Le3dp.dev_addr = dev_addr;
    Le3dp.instance = instance;
    memset((Logitech_E3DP_t *)&Le3dp.report, 0, sizeof(Le3dp.report));
  }
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if ((Le3dp.dev_addr == dev_addr) && (Le3dp.instance == instance)) {
    if (Le3dp.connected) {
      Le3dp.connected = false;
      Le3dp.available = false;
      DBG_printf("Logitech Extreme 3D Pro (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (Le3dp.connected && (Le3dp.dev_addr == dev_addr) && (Le3dp.instance == instance)) {
    if (Le3dp.available) {
      static uint32_t drops = 0;
      DBG_printf("drops=%lu\r\n", ++drops);
    } else {
      memcpy((Logitech_E3DP_t *)&Le3dp.report, report, min(sizeof(Le3dp.report), len));
      Le3dp.report_len = len;
      Le3dp.available = true;
    }
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}
