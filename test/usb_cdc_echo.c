// Dual CDC hardware-test firmware. Build separately; not part of pd_vbus.bin.
#include "usb_cdc.h"

int main(void) {
  static const char command[] = "bootloader";
  uint8_t position[2] = {0, 0};
  uint8_t invalid[2] = {0, 0};
  CDC_init();
  while(1) {
    for(uint8_t port = 0; port < 2; port++) {
      for(uint8_t count = 0; count < 64; count++) {
        char c;
        if(port) {
          if(!CDC2_ready() || !CDC2_tryRead(&c)) break;
          CDC2_tryWrite(c);
        } else {
          if(!CDC_ready() || !CDC_tryRead(&c)) break;
          CDC_tryWrite(c);
        }
        if(c == '\r' || c == '\n') {
          uint8_t boot = !invalid[port] && position[port] == sizeof(command) - 1;
          position[port] = 0;
          invalid[port] = 0;
          if(boot) { DLY_ms(100); RST_bootloader(); }
        } else if(!invalid[port]) {
          if(position[port] < sizeof(command) - 1 && c == command[position[port]]) position[port]++;
          else invalid[port] = 1;
        }
      }
    }
    CDC_flush();
    CDC2_flush();
  }
}
