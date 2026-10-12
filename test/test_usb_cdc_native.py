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
extern USBFSD_TypeDef testUSB;
extern RCC_TypeDef testRCC;
extern AFIO_TypeDef testAFIO;
extern GPIO_TypeDef testGPIO;
extern uint8_t testIRQ;
#define USBFSD (&testUSB)
#define RCC (&testRCC)
#define AFIO (&testAFIO)
#define GPIOC (&testGPIO)
static inline uint8_t NVIC_GetStatusIRQ(IRQn_Type irq) { (void)irq; return testIRQ; }
static inline void NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; testIRQ = 0; }
static inline void NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; testIRQ = 1; }
''')
    executable = build / 'test'
    subprocess.run(['cc', '-std=gnu11', '-O2', '-Wall', '-Wextra',
                    '-Wno-pointer-to-int-cast', '-I', str(build),
                    str(root / 'tools' / 'test_usb_cdc_native.c'),
                    *(str(build / name) for name in ('usb_cdc.c', 'usb_descr.c', 'usb_handler.c')),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
