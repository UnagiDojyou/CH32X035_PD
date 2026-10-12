// Protocol/state regression checks with simulated registers (no USB host needed).
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usb_cdc.h"

USBFSD_TypeDef testUSB;
RCC_TypeDef testRCC;
AFIO_TypeDef testAFIO;
GPIO_TypeDef testGPIO;
uint8_t testIRQ;
void USB_EP0_SETUP(void);
void USBFS_IRQHandler(void);

static void setup(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length) {
  USB_SETUP_REQ data = {type, request, value, value >> 8, index, index >> 8, length, length >> 8};
  memcpy(EP0_buffer, &data, sizeof(data));
  USB_EP0_SETUP();
}

static void configured(void) {
  setup(0, USB_SET_CONFIGURATION, 1, 0, 0);
  assert(USB_Config == 1 && USB_ENUM_OK);
}

int main(void) {
  CDC_init();
  assert(!USB_Config && testIRQ);
  configured();
  // Queue exhaustion must be reported without blocking either port.
  for(unsigned i = 0; i < 64; i++) assert(CDC_tryWrite(i));
  assert(testUSB.UEP2_TX_LEN == 64);
  for(unsigned i = 0; i < 255; i++) assert(CDC_tryWrite(i));
  assert(!CDC_tryWrite('x') && !CDC_ready());
  CDC_write('x');
  assert(CDC_dropped() == 1 && testIRQ);
  assert(CDC2_tryWrite('y'));
  CDC2_flush();
  assert(testUSB.UEP3_TX_LEN == 1 && EP3_buffer[64] == 'y');

  // Reselecting IF1 discards its pending RX/TX and resets DATA toggles only there.
  memcpy(EP2_buffer, "abc", 3);
  testUSB.INT_FG = USBFS_U_TOG_OK;
  testUSB.RX_LEN = 3;
  CDC_EP2_OUT();
  assert(CDC_available() == 3);
  setup(1, USB_SET_INTERFACE, 0, 1, 0);
  assert(!CDC_available() && CDC_ready() && !CDC_dropped());
  assert(testUSB.UEP2_CTRL_H == (USBFS_UEP_AUTO_TOG | USBFS_UEP_T_RES_NAK));
  assert(testUSB.UEP2_TX_LEN == 0 && testUSB.UEP3_TX_LEN == 1);
  CDC_EP3_IN();

  // EP0 OUT data replaces the SETUP packet: keep its original target port.
  setup(0x21, 0x20, 0, 2, 7);
  const uint8_t coding[] = {0x00, 0x84, 0x03, 0x00, 2, 2, 7};
  memcpy(EP0_buffer, coding, 7);
  testUSB.RX_LEN = 7;
  CDC_EP0_OUT();
  assert(CDC2_getBAUD() == 230400 && CDC2_getStopBits() == 2 && CDC2_getParity() == 2);
  assert(CDC_getBAUD() == 115200);
  setup(0xA1, 0x21, 0, 2, 64);
  assert(testUSB.UEP0_TX_LEN == 7 && !memcmp(EP0_buffer, coding, 7));
  setup(0xA1, 0x21, 0, 2, 3);
  assert(testUSB.UEP0_TX_LEN == 3 && !memcmp(EP0_buffer, coding, 3));
  setup(0x21, 0x22, 1, 0, 0);
  setup(0x21, 0x22, 2, 2, 0);
  assert(CDC_getDTR() && !CDC_getRTS() && !CDC2_getDTR() && CDC2_getRTS());
  assert(testUSB.UEP1_TX_LEN == 10 && testUSB.UEP5_TX_LEN == 10);
  assert(EP1_buffer[4] == 0 && EP5_buffer[4] == 2);

  // Unconfiguring must reset all state, including the seven-byte Line Coding.
  setup(0, USB_SET_CONFIGURATION, 0, 0, 0);
  assert(!USB_Config && !USB_ENUM_OK && !CDC_ready() && !CDC2_ready());
  assert(!CDC_tryWrite('x') && !CDC2_tryWrite('y'));
  assert(CDC_getBAUD() == 115200 && CDC2_getBAUD() == 115200);
  assert(!CDC_getDTR() && !CDC2_getRTS());
  configured();

  // Clearing a halt resumes a packet already in DMA, without losing it.
  for(unsigned i = 0; i < 64; i++) assert(CDC_tryWrite(i));
  setup(2, USB_SET_FEATURE, 0, 0x82, 0);
  assert((testUSB.UEP2_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL);
  setup(0x82, USB_GET_STATUS, 0, 0x82, 2);
  assert(EP0_buffer[0] == 1);
  setup(2, USB_CLEAR_FEATURE, 0, 0x82, 0);
  assert((testUSB.UEP2_CTRL_H & (USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG)) == 0);
  assert(testUSB.UEP2_TX_LEN == 64);
  for(unsigned i = 0; i < 64; i++) assert(EP2_buffer[64 + i] == i);
  CDC_EP2_IN();
  assert(testUSB.UEP2_TX_LEN == 0 && (testUSB.UEP2_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK);
  CDC_EP2_IN();
  assert((testUSB.UEP2_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_NAK);

  // A bus-reset interrupt must discard unread data, pending TX, and both states.
  testUSB.INT_FG = USBFS_U_TOG_OK;
  testUSB.RX_LEN = 3;
  CDC_EP3_OUT();
  assert(CDC2_available() == 3);
  assert(CDC2_tryWrite('z'));
  CDC2_flush();
  testUSB.INT_FG = USBFS_UIF_BUS_RST;
  testUSB.INT_ST = 0;
  USBFS_IRQHandler();
  assert(!USB_Config && !USB_ENUM_OK && !CDC_available() && !CDC2_available());
  assert(testUSB.UEP2_TX_LEN == 0 && testUSB.UEP3_TX_LEN == 0);
  assert(CDC_getBAUD() == 115200 && CDC2_getBAUD() == 115200);
  configured();
  assert(CDC_tryWrite('a') && CDC2_tryWrite('b'));
  puts("PASS native: queue overflow, independent ports, SET_INTERFACE, EP0 target/length, halt/ZLP, configuration and bus reset");
}
