#ifndef _BSP_EXT_IO_H_
#define _BSP_EXT_IO_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    union {
        struct {
            uint8_t motion_int:1;
            uint8_t ext_io2:1;
            uint8_t ext_io0:1;
            uint8_t ext_io1:1;
            uint8_t lcd_bl:1;
            uint8_t audio_pa:1;
            uint8_t tp_int:1;
            uint8_t tp_rst:1;
        };
        uint8_t val;
    };
} ext_io_t;

#define BSP_EXT_IO_DEFAULT_CONFIG() {   \
        .motion_int = 1,                \
        .ext_io2 = 1,                   \
        .ext_io0 = 1,                   \
        .ext_io1 = 1,                   \
        .lcd_bl = 0,                    \
        .audio_pa = 0,                  \
        .tp_int = 1,                    \
        .tp_rst = 0,                    \
    }

#define BSP_EXT_IO_DEFAULT_LEVEL() {    \
        .motion_int = 1,                \
        .ext_io2 = 1,                   \
        .ext_io0 = 1,                   \
        .ext_io1 = 1,                   \
        .lcd_bl = 1,                    \
        .audio_pa = 1,                  \
        .tp_int = 1,                    \
        .tp_rst = 1,                    \
    }

#define BSP_EXT_IO_SLEEP_LEVEL() {      \
        .motion_int = 1,                \
        .ext_io2 = 1,                   \
        .ext_io0 = 1,                   \
        .ext_io1 = 1,                   \
        .lcd_bl = 0,                    \
        .audio_pa = 0,                  \
        .tp_int = 1,                    \
        .tp_rst = 0,                    \
    }

#endif
