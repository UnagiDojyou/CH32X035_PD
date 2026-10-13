#!/usr/bin/env python3
"""Run USB protocol/state checks with the real C code and mocked MCU registers."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory(prefix='ch32-cdc-test-') as directory:
    build = Path(directory)
    for header in (root / 'include').glob('*.h'):
        shutil.copy(header, build / header.name)
    shutil.copy(root / 'config.h', build / 'config.h')
    shutil.copy(root / 'usbpd_sink.h', build / 'usbpd_sink.h')
    source = (root / 'usbpd_sink.c').read_text().replace('__attribute__((interrupt))', '')
    source += '''
uint8_t PD_test_write(uint8_t sop, const uint8_t* data, uint16_t length) {
  return PD_logPacket(sop, data, length);
}
// Test-only loader for the real interrupt handler's private DMA buffer.
void PD_test_receive(uint8_t sop, const uint8_t* data, uint16_t length) {
  PD_memcpy(PD_TR_buffer, data, length <= sizeof(PD_TR_buffer) ? length : sizeof(PD_TR_buffer));
  USBPD->STATUS = USBPD_IF_RX_ACT | sop;
  USBPD->BMC_BYTE_CNT = length;
  USBPD->CONTROL = 0;
  USBPD_IRQHandler();
}
'''
    (build / 'usbpd_sink.c').write_text(source)
    for name in ('usb_cdc.c', 'usb_descr.c', 'usb_handler.c'):
        source = (root / 'include' / name).read_text()
        # Host compilers use a different interrupt ABI; only remove that marker.
        (build / name).write_text(source.replace('__attribute__((interrupt))', ''))
    (build / 'system.h').write_text('''#pragma once
#include "ch32x035.h"
#define SYS_USE_VECTORS 1
#undef USBFSD
#undef RCC
#undef AFIO
#undef GPIOC
#undef USBPD
extern USBFSD_TypeDef testUSB;
extern USBPD_TypeDef testUSBPD;
extern RCC_TypeDef testRCC;
extern AFIO_TypeDef testAFIO;
extern GPIO_TypeDef testGPIO;
extern uint8_t testIRQ;
extern uint8_t testPDIRQ;
#define USBFSD (&testUSB)
#define RCC (&testRCC)
#define AFIO (&testAFIO)
#define GPIOC (&testGPIO)
#define USBPD (&testUSBPD)
#define STK SysTick
#define DLY_MS_TIME 48000
static inline void DLY_ms(uint32_t n) { (void)n; }
static inline void DLY_us(uint32_t n) { (void)n; }
static inline void NVIC_SetPriority(IRQn_Type irq, uint8_t priority) { (void)irq; (void)priority; }
static inline uint8_t NVIC_GetStatusIRQ(IRQn_Type irq) { return irq == USBPD_IRQn ? testPDIRQ : testIRQ; }
static inline void NVIC_DisableIRQ(IRQn_Type irq) { if(irq == USBPD_IRQn) testPDIRQ = 0; else testIRQ = 0; }
static inline void NVIC_EnableIRQ(IRQn_Type irq) { if(irq == USBPD_IRQn) testPDIRQ = 1; else testIRQ = 1; }
''')
    for cdc_only, default_sizes in ((True, False), (False, False), (True, True)):
        if default_sizes:
            config = (root / 'config.h').read_text()
            config = '\n'.join(line for line in config.splitlines() if not line.startswith('#define CDC2_TX_BUFFER_SIZE'))
            (build / 'config.h').write_text(config)
        executable = build / ('test-cdc' if cdc_only else 'test-pd')
        names = ['usb_cdc.c', 'usb_descr.c', 'usb_handler.c']
        options = ['-DCDC_TEST_ONLY'] if cdc_only else []
        if not cdc_only:
            names.append('usbpd_sink.c')
        subprocess.run(['cc', '-std=gnu11', '-O2', '-Wall', '-Wextra',
                        '-Wno-pointer-to-int-cast', '-DF_CPU=48000000',
                        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-I', str(build),
                        *options, str(root / 'test' / 'test_usb_cdc_native.c'),
                        *(str(build / name) for name in names),
                        '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
