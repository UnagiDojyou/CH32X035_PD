// Protocol/state regression checks with simulated registers (no USB host needed).
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usb_cdc.h"

USBFSD_TypeDef testUSB;
RCC_TypeDef testRCC;
USBPD_TypeDef testUSBPD;
AFIO_TypeDef testAFIO;
GPIO_TypeDef testGPIO;
uint8_t testIRQ;
uint8_t testPDIRQ;
void USB_EP0_SETUP(void);
void USBFS_IRQHandler(void);
uint8_t PD_test_write(uint8_t sop, const uint8_t* data, uint16_t length);
void PD_test_receive(uint8_t sop, const uint8_t* data, uint16_t length);

static void setup(uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length) {
  USB_SETUP_REQ data = {type, request, value, value >> 8, index, index >> 8, length, length >> 8};
  memcpy(EP0_buffer, &data, sizeof(data));
  USB_EP0_SETUP();
}

static void configured(void) {
  setup(0, USB_SET_CONFIGURATION, 1, 0, 0);
  assert(USB_Config == 1 && USB_ENUM_OK);
}

static void drain(uint8_t port, const uint8_t* data, unsigned length) {
  unsigned received = 0;
  volatile uint16_t* control = port ? &testUSB.UEP3_CTRL_H : &testUSB.UEP2_CTRL_H;
  volatile uint16_t* size = port ? &testUSB.UEP3_TX_LEN : &testUSB.UEP2_TX_LEN;
  uint8_t* dma = (port ? EP3_buffer : EP2_buffer) + 64;
  while((*control & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK) {
    assert(received + *size <= length);
    assert(!memcmp(data + received, dma, *size));
    received += *size;
    if(port) CDC_EP3_IN(); else CDC_EP2_IN();
  }
  assert(received == length);
}

static void bulk_writes(void) {
  uint8_t data[CDC_TX_BUFFER_SIZE > CDC2_TX_BUFFER_SIZE ? CDC_TX_BUFFER_SIZE : CDC2_TX_BUFFER_SIZE];
  for(unsigned i = 0; i < sizeof(data); i++) data[i] = i * 13 + 7;
  for(uint8_t port = 0; port < 2; port++) {
    uint8_t (*write)(const void*, uint16_t) = port ? CDC2_writeBuffer : CDC_writeBuffer;
    uint8_t (*tryWrite)(const void*, uint16_t) = port ? CDC2_tryWriteBuffer : CDC_tryWriteBuffer;
    uint8_t (*reserve)(uint16_t, CDC_TX_RESERVATION*) = port ? CDC2_reserveWrite : CDC_reserveWrite;
    uint16_t (*available)(void) = port ? CDC2_writeAvailable : CDC_writeAvailable;
    uint32_t (*dropped)(void) = port ? CDC2_dropped : CDC_dropped;
    uint16_t capacity = (port ? CDC2_TX_BUFFER_SIZE : CDC_TX_BUFFER_SIZE) - 1;
    uint8_t endpoint = port ? 0x83 : 0x82;
    configured();
    testIRQ = testPDIRQ = 1;
    const unsigned lengths[] = {1, 63, 64, 65, capacity, capacity};
    for(unsigned i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
      assert(available() == capacity);
      assert(tryWrite(data, lengths[i]));
      drain(port, data, lengths[i]);
    }
    CDC_TX_RESERVATION tx;
    assert(reserve(17, &tx));
    assert(!testIRQ);
#ifdef CDC_TEST_ONLY
    assert(testPDIRQ);       // Default hooks do not know about application IRQs.
#else
    assert(!testPDIRQ);      // PD application overrides both hooks.
#endif
    memset(tx.data[0], 0xaa, tx.length[0]);
    memset(tx.data[1], 0xaa, tx.length[1]);
    CDC_cancelWrite(&tx);
    CDC_cancelWrite(&tx);   // Repeated cancel is harmless.
    assert(testIRQ && testPDIRQ && available() == capacity && !dropped());
    assert(reserve(17, &tx));
    memcpy(tx.data[0], data, tx.length[0]);
    memcpy(tx.data[1], data + tx.length[0], tx.length[1]);
    CDC_commitWrite(&tx);
    CDC_commitWrite(&tx);   // Repeated commit must not publish twice.
    drain(port, data, 17);
    // Fill while halted, then reject writes atomically; try API has no drops.
    setup(2, USB_SET_FEATURE, 0, endpoint, 0);
    assert(write(data, capacity));
    assert(!available());
    assert(!tryWrite(data, 3) && !dropped());
    assert(!write(data, 3) && dropped() == 3);
    setup(2, USB_CLEAR_FEATURE, 0, endpoint, 0);
    drain(port, data, capacity);
    assert(write(NULL, 0) && !write(NULL, 1));
    setup(0, USB_SET_CONFIGURATION, 0, 0, 0);
    assert(!available());
    assert(!tryWrite(data, 3) && !dropped());
    assert(!write(data, 3) && dropped() == 3);
  }
  puts("PASS bulk CDC1/CDC2: atomic writes, free capacity, split reservations, commit/cancel, full/unconfigured and drop policies");
}

#ifndef CDC_TEST_ONLY
static void pd_packets(void) {
  uint8_t data[264], wire[265];
  const unsigned lengths[] = {1, 6, 34, 63, 64, 65, 128, 255, 264};
  for(unsigned i = 0; i < sizeof(data); i++) data[i] = i * 37 + 19;
  configured();
  testPDIRQ = 1;
  for(unsigned sop = 1; sop <= 3; sop++) {
    for(unsigned n = 0; n < sizeof(lengths) / sizeof(lengths[0]); n++) {
      unsigned length = lengths[n], received = 0;
      assert(PD_test_write(sop, data, length));
      assert(testPDIRQ && testIRQ);
      while((testUSB.UEP3_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK) {
        unsigned count = testUSB.UEP3_TX_LEN;
        assert(received + count <= length + 1);
        memcpy(wire + received, EP3_buffer + 64, count);
        received += count;
        CDC_EP3_IN();
      }
      assert(received == length + 1 && wire[0] >> 6 == sop);
      assert(wire[0] == (sop << 6));
      assert(!memcmp(wire + 1, data, length));
    }
  }
  // A maximum record fits when halted; insufficient space drops the whole next
  // record. Clear halt must send exactly the first record, without a fragment.
  setup(2, USB_SET_FEATURE, 0, 0x83, 0);
  assert(PD_test_write(3, data, 264));
  assert(!CDC2_ready());
  uint32_t dropped = CDC2_dropped();
  assert(!PD_test_write(1, data, 6));
  assert(CDC2_dropped() == dropped + 7);
  assert(CDC_tryWrite('a'));  // CDC1 is still independent.
  setup(2, USB_CLEAR_FEATURE, 0, 0x83, 0);
  unsigned received = 0;
  while((testUSB.UEP3_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_ACK) {
    unsigned count = testUSB.UEP3_TX_LEN;
    assert(received + count <= sizeof(wire));
    memcpy(wire + received, EP3_buffer + 64, count);
    received += count;
    CDC_EP3_IN();
  }
  assert(received == 265 && wire[0] >> 6 == 3);
  assert(wire[0] == 0xc0 && !memcmp(wire + 1, data, sizeof(data)));
  assert(!PD_test_write(0, data, 6));
  assert(!PD_test_write(4, data, 6));
  assert(!PD_test_write(1, data, 0));
  assert(!PD_test_write(1, data, 265));
  // Preserve the original IRQ masks, including the case USB is already masked.
  testIRQ = testPDIRQ = 0;
  assert(PD_test_write(1, data, 6));
  assert(!testIRQ && !testPDIRQ);
  testIRQ = testPDIRQ = 1;
  setup(1, USB_SET_INTERFACE, 0, 3, 0);
  assert(CDC2_ready() && !CDC2_dropped() && !testUSB.UEP3_TX_LEN);
  setup(0, USB_SET_CONFIGURATION, 0, 0, 0);
  assert(!PD_test_write(1, data, 6) && CDC2_dropped() == 7);
  puts("PASS PD: SOP/SOP'/SOP'' aligned prefix/raw data, lengths 1..264, ring wrap, whole-packet overflow, halt, reset and IRQ masks");
}

static void pd_receive_interrupt(void) {
  // Accept is deliberately not GoodCRC: the SOP path overwrites the DMA header
  // to transmit GoodCRC. CDC2 must retain the original header and all CRC bytes.
  const uint8_t data[6] = {0x83, 0x00, 0x12, 0x34, 0x56, 0x78};
  for(uint8_t sop = 1; sop <= 3; sop++) {
    configured();
    PD_test_receive(sop, data, sizeof(data));
    assert(testUSB.UEP3_TX_LEN == 7);
    const uint8_t* wire = EP3_buffer + 64;
    assert(wire[0] == (sop << 6));
    assert(!memcmp(wire + 1, data, sizeof(data)));
    if(sop == 1) assert(testUSBPD.CONTROL & USBPD_PD_TX_EN);
    else {
      assert(!(testUSBPD.CONTROL & USBPD_PD_TX_EN));
      assert(testUSBPD.CONTROL & USBPD_BMC_START);
    }
  }
  configured();
  PD_test_receive(0, data, sizeof(data));
  assert(!testUSB.UEP3_TX_LEN);
  puts("PASS real PD ISR: copies before GoodCRC overwrite, SOP'/SOP'' capture/re-arm, invalid SOP ignored");
}

#endif

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
  bulk_writes();
#ifndef CDC_TEST_ONLY
  pd_packets();
  pd_receive_interrupt();
#endif
}
