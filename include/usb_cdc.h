// ===================================================================================
// Dual USB CDC Functions for CH32X035/X034/X033                             * v2.0 *
// ===================================================================================
//
// Functions below use the first port; CDC2_* uses the second port.
// Call from main code; USB callbacks share state with these functions.
// Functions available:
// --------------------
// CDC_init()               setup USB CDC
// CDC_read()               read single character from receive buffer
// CDC_write(c)             write single character to transmit buffer
// CDC_flush()              flush transmit buffer
// CDC_writeflush(c)        write & flush character
// CDC_newline()            newline and flush
//
// CDC_available()          check number of bytes in the receive buffer
// CDC_ready()              check if transmit buffer is ready to be written
//
// CDC_getDTR()             get DTR flag
// CDC_getRTS()             get RTS flag
// CDC_getBAUD()            get BAUD rate
// CDC_getParity()          get parity (0:none, 1:odd, 2:even, 3:mark, 4:space)
// CDC_getStopBits()        get number of stop bits (0:1bit, 1:1.5bits, 2:2bits)
// CDC_getDataBits()        get number of data bits (5, 6, 7, 8, or 16)
//
// If print functions are activated (see below, print.h must be included):
// -----------------------------------------------------------------------
// CDC_printf(f, ...)       printf (supports %s, %c, %d, %u, %x, %b, %02d, %%)
// CDC_printD(n)            print decimal value
// CDC_printW(n)            print 32-bit hex word value
// CDC_printH(n)            print 16-bit hex half-word value
// CDC_printB(n)            print  8-bit hex byte value
// CDC_printS(s)            print string
// CDC_print(s)             print string (alias)
// CDC_println(s)           print string with newline and flush
//
// 2023 by Stefan Wagner:   https://github.com/wagiminator
// 2026 by Unagi Dojyou:    https://unagidojyou.com

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"
#include "usb_descr.h"
#include "usb_handler.h"

// ===================================================================================
// CDC Parameters
// ===================================================================================
#define CDC_PRINT       1         // 1: include print functions (needs print.h)

// ===================================================================================
// CDC Functions
// ===================================================================================
void CDC_init(void);              // setup USB-CDC
void CDC_flush(void);             // schedule queued TX data (does not wait)
char CDC_read(void);              // wait for and read one RX character
void CDC_write(char c);           // queue one TX character, drop if full
uint8_t CDC_available(void);      // number of buffered RX bytes
uint8_t CDC_ready(void);          // TX queue has room and USB is configured

// Writes never wait for the host. The 255-byte queue drops new bytes when full.
// Use tryWrite for lossless streams: retry later without consuming the source byte.
uint8_t CDC_tryWrite(char c);       // return 1 if queued, 0 if full/unconfigured
uint8_t CDC_tryRead(char* c);       // return 1 if read, 0 if empty/unconfigured
uint32_t CDC_dropped(void);        // dropped bytes since USB reset/configuration

// Second CDC port (USB interfaces 2 and 3)
void CDC2_flush(void);
char CDC2_read(void);              // like CDC_read, waits for a byte
void CDC2_write(char c);
uint8_t CDC2_available(void);
uint8_t CDC2_ready(void);
uint8_t CDC2_tryWrite(char c);
uint8_t CDC2_tryRead(char* c);
uint32_t CDC2_dropped(void);

#define CDC2_writeflush(c)        {CDC2_write(c);CDC2_flush();}
#define CDC2_newline()            {CDC2_write('\n');CDC2_flush();}

#define CDC_writeflush(c)         {CDC_write(c);CDC_flush();}     // write & flush char
#define CDC_newline()             {CDC_write('\n'); CDC_flush();} // newline and flush

// ===================================================================================
// CDC Control Line State
// ===================================================================================
extern volatile uint8_t CDC_controlLineStates[2];
#define CDC_controlLineState CDC_controlLineStates[0]
#define CDC2_controlLineState CDC_controlLineStates[1]

#define CDC_DTR_flag    (CDC_controlLineState & 1)          // DTR flag
#define CDC_RTS_flag    ((CDC_controlLineState >> 1) & 1)   // RTS flag
#define CDC_getDTR()    (CDC_DTR_flag)                      // get DTR flag
#define CDC_getRTS()    (CDC_RTS_flag)                      // get RTS flag

// ===================================================================================
// CDC Line Coding
// ===================================================================================
typedef struct __attribute__((packed)) {
  uint32_t baudrate;              // baud rate
  uint8_t  stopbits;              // number of stopbits (0:1bit,1:1.5bits,2:2bits)
  uint8_t  parity;                // parity (0:none,1:odd,2:even,3:mark,4:space)
  uint8_t  databits;              // number of data bits (5,6,7,8 or 16)
} CDC_LINE_CODING_TYPE;

extern volatile CDC_LINE_CODING_TYPE CDC_lineCodings[2];
#define CDC_lineCoding  CDC_lineCodings[0]
#define CDC2_lineCoding CDC_lineCodings[1]
#define CDC_getBAUD()       (CDC_lineCoding.baudrate)
#define CDC_getParity()     (CDC_lineCoding.parity)
#define CDC_getStopBits()   (CDC_lineCoding.stopbits)
#define CDC_getDataBits()   (CDC_lineCoding.databits)

#define CDC2_getDTR()       (CDC2_controlLineState & 1)
#define CDC2_getRTS()       ((CDC2_controlLineState >> 1) & 1)
#define CDC2_getBAUD()      (CDC2_lineCoding.baudrate)
#define CDC2_getParity()    (CDC2_lineCoding.parity)
#define CDC2_getStopBits()  (CDC2_lineCoding.stopbits)
#define CDC2_getDataBits()  (CDC2_lineCoding.databits)

// ===================================================================================
// Additional Print Functions (if activated, see above)
// ===================================================================================
#if CDC_PRINT == 1
#include "print.h"
#define CDC_printD(n)         printD(CDC_write, n)    // print decimal as string
#define CDC_printW(n)         printW(CDC_write, n)    // print word as string
#define CDC_printH(n)         printH(CDC_write, n)    // print half-word as string
#define CDC_printB(n)         printB(CDC_write, n)    // print byte as string
#define CDC_printS(s)         printS(CDC_write, s)    // print string
#define CDC_println(s)        {println(CDC_write, s); CDC_flush();}
#define CDC2_printD(n)        printD(CDC2_write, n)
#define CDC2_printW(n)        printW(CDC2_write, n)
#define CDC2_printH(n)        printH(CDC2_write, n)
#define CDC2_printB(n)        printB(CDC2_write, n)
#define CDC2_printS(s)        printS(CDC2_write, s)
#define CDC2_println(s)       {println(CDC2_write, s); CDC2_flush();}
#define CDC2_print            CDC2_printS
#define CDC2_printf(f, ...)   {printF(CDC2_write, f, ##__VA_ARGS__); CDC2_flush();}
#define CDC_print             CDC_printS              // alias
#define CDC_printf(f, ...)    {printF(CDC_write, f, ##__VA_ARGS__); CDC_flush();}
#endif

#ifdef __cplusplus
}
#endif
