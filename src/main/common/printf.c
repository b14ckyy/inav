/*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "printf.h"

#include "io/serial.h"


#ifdef REQUIRE_PRINTF_LONG_SUPPORT
#include "typeconversion.h"
#define MAX UINT64_MAX
#else
#define MAX UINT32_MAX
#endif

static serialPort_t *printfSerialPort;

#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT

typedef void (*putcf) (void *, char);
static putcf stdout_putf;
static void *stdout_putp;

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, const void *end, putcf putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char pr = 0;
    if (n < 0) {
        pr = 1;
        n = -n;
    }
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    if (pr == 0) {
        while (n-- > 0) {
            if (putp < end) {
                putf(putp, fc);
            }
            written++;
        }
    }
    while ((ch = *bf++)) {
        if (putp < end) {
            putf(putp, ch);
        }
        written++;
    }
    if (pr == 1) {
        while (n-- > 0) {
            if (putp < end) {
                putf(putp, fc);
            }
            written++;
        }
    }
    return written;
}

int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    return tfp_nformat(putp, -1, putf, fmt, va);
}

// return number of bytes written
int tfp_nformat(void *putp, int size, void (*putf) (void *, char), const char *fmt, va_list va)
{
    char bf[12];
    int written = 0;
    char ch;

    const void *end = size < 0 ? (void*)MAX : ((char *)putp + size - 1);

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            if (putp < end) {
                putf(putp, ch);
            }
            written++;
        } else {
            char lz = 0;
            char pr = 0; // padding at the right?
#ifdef REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '-') {
                ch = *(fmt++);
                pr = 1;
            }
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
                if (pr) {
                    w = -w;
                }
            }
#ifdef REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0:
                goto abort;
            case 'u':{
#ifdef REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    written += putchw(putp, end, putf, w, lz, bf);
                    break;
                }
            case 'i':
            case 'd':{
#ifdef REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        li2a(va_arg(va, unsigned long int), bf);
                    else
#endif
                        i2a(va_arg(va, int), bf);
                    written += putchw(putp, end, putf, w, lz, bf);
                    break;
                }
            case 'x':
            case 'X':
#ifdef REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                written += putchw(putp, end, putf, w, lz, bf);
                break;
            case 'c':
                if (putp < end) {
                    putf(putp, (char) (va_arg(va, int)));
                }
                written++;
                break;
            case 's':
                written += putchw(putp, end, putf, w, 0, va_arg(va, char *));
                break;
            case '%':
                if (putp < end) {
                    putf(putp, ch);
                }
                written++;
                break;
            case 'n':
                *va_arg(va, int*) = written;
                break;
            case 'f':
                ftoa(va_arg(va, double), bf);
                written += putchw(putp, end, putf, w, lz, bf);
                break;
            default:
                break;
            }
        }
    }
abort:
    return written;
}

void init_printf(void *putp, void (*putf) (void *, char))
{
    stdout_putf = putf;
    stdout_putp = putp;
}

int tfp_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int written = tfp_format(stdout_putp, stdout_putf, fmt, va);
    va_end(va);
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    return written;
}

static void putcp(void *p, char c)
{
    *(*((char **) p))++ = c;
}

int tfp_sprintf(char *s, const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    int written = tfp_vsprintf(s, fmt, va);
    va_end(va);
    return written;
}

int tfp_snprintf(char *s, int size, const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    int written = tfp_vsnprintf(s, size, fmt, va);
    va_end(va);
    return written;
}

int tfp_vsprintf(char *s, const char *fmt, va_list va)
{
    return tfp_vsnprintf(s, -1, fmt, va);
}

int tfp_vsnprintf(char *s, int size, const char *fmt, va_list va)
{
    int written = tfp_nformat(&s, size, putcp, fmt, va);
    putcp(&s, 0);
    return written;
}


static void _putc(void *p, char c)
{
    UNUSED(p);
    serialWrite(printfSerialPort, c);
}

void printfSupportInit(void)
{
    init_printf(NULL, _putc);
}

#else

// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    serialWrite(printfSerialPort, c);
    return c;
}

void printfSupportInit(void)
{
    // Nothing to do
}
#endif

void setPrintfSerialPort(serialPort_t *serialPort)
{
    printfSerialPort = serialPort;
}
