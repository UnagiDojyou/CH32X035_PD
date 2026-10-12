// ===================================================================================
// Dual USB CDC Functions for CH32X035/X034/X033
// ===================================================================================

#include "usb_cdc.h"

// ===================================================================================
// Variables and Defines
// ===================================================================================
#define CDC_PORTS                 2
#define CDC_PACKET_SIZE           64
#define CDC_WRITE_BUF_SIZE        256      // 255 usable bytes, uint8_t pointers wrap
#define SET_LINE_CODING           0x20
#define GET_LINE_CODING           0x21
#define SET_CONTROL_LINE_STATE    0x22

volatile CDC_LINE_CODING_TYPE CDC_lineCodings[CDC_PORTS];
volatile uint8_t CDC_controlLineStates[CDC_PORTS];
_Static_assert(sizeof(CDC_LINE_CODING_TYPE) == 7, "CDC line coding must be 7 bytes");

typedef struct {
  uint8_t readByteCount;
  uint8_t readPointer;
  uint8_t writePointer;
  uint8_t writeReadPointer;
  uint8_t writeBusyFlag;
  uint8_t flushPending;
  uint8_t writeZLP;
  uint8_t notifyBusyFlag;
  uint8_t notifyPending;
  uint32_t dropped;
} CDC_STATE_TYPE;

static volatile CDC_STATE_TYPE CDC_state[CDC_PORTS];
static uint8_t CDC_writeBuffer[CDC_PORTS][CDC_WRITE_BUF_SIZE];
static uint8_t CDC_setupPort;               // survives overwriting EP0 with OUT data

// Endpoints 2 and 3 use OUT at DMA+0 and IN at DMA+64.
static uint8_t* const CDC_buffers[CDC_PORTS] = {EP2_buffer, EP3_buffer};
static volatile uint16_t* const CDC_controls[CDC_PORTS] = {
  &USBFSD->UEP2_CTRL_H, &USBFSD->UEP3_CTRL_H
};
static volatile uint16_t* const CDC_lengths[CDC_PORTS] = {
  &USBFSD->UEP2_TX_LEN, &USBFSD->UEP3_TX_LEN
};
static uint8_t* const CDC_notifyBuffers[CDC_PORTS] = {EP1_buffer, EP5_buffer};
static volatile uint16_t* const CDC_notifyControls[CDC_PORTS] = {
  &USBFSD->UEP1_CTRL_H, &USBFSD->UEP5_CTRL_H
};
static volatile uint16_t* const CDC_notifyLengths[CDC_PORTS] = {
  &USBFSD->UEP1_TX_LEN, &USBFSD->UEP5_TX_LEN
};

// Only USB shares these buffers with main code; leave PD interrupts enabled.
static uint8_t CDC_lock(void) {
  uint8_t enabled = NVIC_GetStatusIRQ(USBFS_IRQn);
  NVIC_DisableIRQ(USBFS_IRQn);
  asm volatile("" ::: "memory");
  return enabled;
}

static void CDC_unlock(uint8_t enabled) {
  asm volatile("" ::: "memory");
  if(enabled) NVIC_EnableIRQ(USBFS_IRQn);
}

// Caller runs in the USB ISR or with the USB interrupt disabled.
static void CDC_send(uint8_t port) {
  volatile CDC_STATE_TYPE* state = &CDC_state[port];
  uint8_t count = state->writePointer - state->writeReadPointer;
  uint8_t len;
  if(!USB_Config || state->writeBusyFlag ||
     (*CDC_controls[port] & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) return;
  if(count && (count >= CDC_PACKET_SIZE || state->flushPending)) {
    len = count > CDC_PACKET_SIZE ? CDC_PACKET_SIZE : count;
    for(uint8_t i = 0; i < len; i++)
      CDC_buffers[port][64 + i] = CDC_writeBuffer[port][state->writeReadPointer++];
  } else if(state->writeZLP) len = 0;        // terminate a full-size USB transfer
  else {
    if(!count) state->flushPending = 0;
    return;
  }
  state->writeZLP = (len == CDC_PACKET_SIZE);
  state->writeBusyFlag = 1;                 // publish state before enabling IN
  *CDC_lengths[port] = len;
  *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

// SERIAL_STATE: this virtual port is ready (DCD/DSR set). No UART error flags.
static void CDC_notify(uint8_t port) {
  volatile CDC_STATE_TYPE* state = &CDC_state[port];
  if(!USB_Config || state->notifyBusyFlag || !state->notifyPending ||
     (*CDC_notifyControls[port] & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) return;
  uint8_t* data = CDC_notifyBuffers[port];
  data[0] = 0xA1;                           // class notification, interface recipient
  data[1] = 0x20;                           // SERIAL_STATE
  data[2] = data[3] = 0;                    // wValue
  data[4] = port * 2; data[5] = 0;          // wIndex: control interface
  data[6] = 2; data[7] = 0;                 // wLength
  data[8] = 3; data[9] = 0;                 // DCD and DSR
  state->notifyPending = 0;
  state->notifyBusyFlag = 1;
  *CDC_notifyLengths[port] = 10;
  *CDC_notifyControls[port] = (*CDC_notifyControls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

// ===================================================================================
// Front End Functions (main code only)
// ===================================================================================
void CDC_init(void) {
  USB_init();
}

static uint8_t CDC_portReady(uint8_t port) {
  return USB_Config && (uint8_t)(CDC_state[port].writePointer + 1) != CDC_state[port].writeReadPointer;
}

static uint8_t CDC_queueWrite(uint8_t port, char c) {
  uint8_t ready = CDC_portReady(port);
  if(ready) {
    CDC_writeBuffer[port][CDC_state[port].writePointer++] = c;
    CDC_send(port);
  }
  return ready;
}

static uint8_t CDC_portTryWrite(uint8_t port, char c) {
  uint8_t enabled = CDC_lock();
  uint8_t ready = CDC_queueWrite(port, c);
  CDC_unlock(enabled);
  return ready;
}

static void CDC_portWrite(uint8_t port, char c) {
  uint8_t enabled = CDC_lock();
  if(!CDC_queueWrite(port, c)) CDC_state[port].dropped++;
  CDC_unlock(enabled);
}

static void CDC_portFlush(uint8_t port) {
  uint8_t enabled = CDC_lock();
  CDC_state[port].flushPending = 1;
  CDC_send(port);
  CDC_unlock(enabled);
}

static uint8_t CDC_portTryRead(uint8_t port, char* c) {
  uint8_t enabled = CDC_lock();
  uint8_t available = USB_Config && CDC_state[port].readByteCount;
  if(available) {
    *c = CDC_buffers[port][CDC_state[port].readPointer++];
    if(--CDC_state[port].readByteCount == 0 &&
       (*CDC_controls[port] & USBFS_UEP_R_RES_MASK) != USBFS_UEP_R_RES_STALL)
      *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
  }
  CDC_unlock(enabled);
  return available;
}

uint8_t CDC_available(void)  { return CDC_state[0].readByteCount; }
uint8_t CDC2_available(void) { return CDC_state[1].readByteCount; }
uint8_t CDC_ready(void)      { return CDC_portReady(0); }
uint8_t CDC2_ready(void)     { return CDC_portReady(1); }
uint8_t CDC_tryWrite(char c) { return CDC_portTryWrite(0, c); }
uint8_t CDC2_tryWrite(char c){ return CDC_portTryWrite(1, c); }
uint8_t CDC_tryRead(char* c) { return CDC_portTryRead(0, c); }
uint8_t CDC2_tryRead(char* c){ return CDC_portTryRead(1, c); }
uint32_t CDC_dropped(void)  { return CDC_state[0].dropped; }
uint32_t CDC2_dropped(void) { return CDC_state[1].dropped; }
void CDC_write(char c)      { CDC_portWrite(0, c); }
void CDC2_write(char c)     { CDC_portWrite(1, c); }
void CDC_flush(void)        { CDC_portFlush(0); }
void CDC2_flush(void)       { CDC_portFlush(1); }
char CDC_read(void)         { char c; while(!CDC_tryRead(&c)); return c; }
char CDC2_read(void)        { char c; while(!CDC2_tryRead(&c)); return c; }

// ===================================================================================
// CDC-Specific USB Handler Functions
// ===================================================================================
// SET_INTERFACE also selects alternate setting 0 again: reset that interface's
// DATA toggles and pending transfers, without changing the other interface/port.
void CDC_EP_resetInterface(uint8_t interface) {
  uint8_t port = interface / 2;
  volatile CDC_STATE_TYPE* state = &CDC_state[port];
  if(interface & 1) {
    *CDC_controls[port] = USBFS_UEP_AUTO_TOG | USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
    *CDC_lengths[port] = 0;
    state->readByteCount = state->readPointer = 0;
    state->writePointer = state->writeReadPointer = 0;
    state->writeBusyFlag = state->flushPending = state->writeZLP = 0;
    state->dropped = 0;
  } else {
    *CDC_notifyControls[port] = USBFS_UEP_AUTO_TOG | USBFS_UEP_T_RES_NAK;
    *CDC_notifyLengths[port] = 0;
    state->notifyBusyFlag = state->notifyPending = 0;
  }
}

void CDC_EP_init(void) {
  USBFSD->UEP1_DMA    = (uint32_t)EP1_buffer;
  USBFSD->UEP2_DMA    = (uint32_t)EP2_buffer;
  USBFSD->UEP3_DMA    = (uint32_t)EP3_buffer;
  USBFSD->UEP5_DMA    = (uint32_t)EP5_buffer;
  USBFSD->UEP4_1_MOD  = USBFS_UEP1_TX_EN;
  USBFSD->UEP2_3_MOD  = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN
                      | USBFS_UEP3_RX_EN | USBFS_UEP3_TX_EN;
  USBFSD->UEP567_MOD  = USBFS_UEP5_TX_EN;
  for(uint8_t port = 0; port < CDC_PORTS; port++) {
    *CDC_controls[port] = USBFS_UEP_AUTO_TOG | USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
    *CDC_notifyControls[port] = USBFS_UEP_AUTO_TOG | USBFS_UEP_T_RES_NAK;
    *CDC_lengths[port] = *CDC_notifyLengths[port] = 0;
    volatile uint8_t* state = (volatile uint8_t*)&CDC_state[port];
    for(uint8_t i = 0; i < sizeof(CDC_STATE_TYPE); i++) state[i] = 0;
    CDC_controlLineStates[port] = 0;
    CDC_lineCodings[port].baudrate = 115200;
    CDC_lineCodings[port].stopbits = 0;
    CDC_lineCodings[port].parity = 0;
    CDC_lineCodings[port].databits = 8;
  }
  CDC_setupPort = 0xff;
}

uint8_t CDC_control(void) {
  CDC_setupPort = 0xff;
  if(!USB_Config || (USB_SetupTyp & USB_REQ_RECIP_MASK) != USB_REQ_RECIP_INTERF ||
     USB_SetupBuf->wIndexH || (USB_SetupBuf->wIndexL != 0 && USB_SetupBuf->wIndexL != 2)) return 0xff;
  uint8_t port = USB_SetupBuf->wIndexL / 2;
  switch(USB_SetupReq) {
    case GET_LINE_CODING:
      if(USB_SetupTyp != 0xA1 || USB_SetupBuf->wValueL || USB_SetupBuf->wValueH) return 0xff;
      if(USB_SetupLen > 7) USB_SetupLen = 7;
      USB_pDescr = (uint8_t*)&CDC_lineCodings[port];
      USB_EP0_copyDescr(USB_SetupLen);
      return USB_SetupLen;
    case SET_CONTROL_LINE_STATE:
      if(USB_SetupTyp != 0x21 || USB_SetupLen || USB_SetupBuf->wValueH ||
         (USB_SetupBuf->wValueL & ~3)) return 0xff;
      CDC_controlLineStates[port] = USB_SetupBuf->wValueL;
      CDC_state[port].notifyPending = 1;
      CDC_notify(port);
      return 0;
    case SET_LINE_CODING:
      if(USB_SetupTyp != 0x21 || USB_SetupLen != 7 || USB_SetupBuf->wValueL || USB_SetupBuf->wValueH) return 0xff;
      CDC_setupPort = port;                 // save before EP0 receives data
      return 0;
    default:
      return 0xff;
  }
}

void CDC_EP0_OUT(void) {
  if(USB_SetupReq == SET_LINE_CODING && CDC_setupPort < CDC_PORTS) {
    if(USBFSD->RX_LEN != 7) {
      CDC_setupPort = 0xff;
      USBFSD->UEP0_CTRL_H = USBFS_UEP_T_RES_STALL | USBFS_UEP_R_RES_STALL;
      return;
    }
    for(uint8_t i = 0; i < 7; i++) ((volatile uint8_t*)&CDC_lineCodings[CDC_setupPort])[i] = EP0_buffer[i];
    CDC_setupPort = 0xff;
    USB_SetupLen = 0;
  }
  USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK | USBFS_UEP_R_RES_ACK;
}

static void CDC_portIN(uint8_t port) {
  *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
  CDC_state[port].writeBusyFlag = 0;
  CDC_send(port);
}

static void CDC_portOUT(uint8_t port) {
  if((USBFSD->INT_FG & USBFS_U_TOG_OK) && USBFSD->RX_LEN) {
    *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    CDC_state[port].readByteCount = USBFSD->RX_LEN;
    CDC_state[port].readPointer = 0;
  }
}

static void CDC_notifyIN(uint8_t port) {
  *CDC_notifyControls[port] = (*CDC_notifyControls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
  CDC_state[port].notifyBusyFlag = 0;
  CDC_notify(port);
}

// Resume a packet already in DMA when the host clears ENDPOINT_HALT.
void CDC_EP_clearHalt(uint8_t address) {
  uint8_t port;
  if(address == 0x82 || address == 0x83) {
    port = (address == 0x83);
    if(CDC_state[port].writeBusyFlag)
      *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    else CDC_send(port);
  } else if(address == 0x02 || address == 0x03) {
    port = (address == 0x03);
    if(CDC_state[port].readByteCount)
      *CDC_controls[port] = (*CDC_controls[port] & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
  } else if(address == 0x81 || address == 0x85) {
    port = (address == 0x85);
    if(CDC_state[port].notifyBusyFlag)
      *CDC_notifyControls[port] = (*CDC_notifyControls[port] & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    else CDC_notify(port);
  }
}

void CDC_EP1_IN(void)  { CDC_notifyIN(0); }
void CDC_EP2_IN(void)  { CDC_portIN(0); }
void CDC_EP2_OUT(void) { CDC_portOUT(0); }
void CDC_EP3_IN(void)  { CDC_portIN(1); }
void CDC_EP3_OUT(void) { CDC_portOUT(1); }
void CDC_EP5_IN(void)  { CDC_notifyIN(1); }
