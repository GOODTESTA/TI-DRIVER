/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TI_MCAN_RW_H
#define TI_MCAN_RW_H

#include <drivers/mcan.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HeapP.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    uint32_t id;
    void (*func)(uint32_t len, uint8_t *byte);
} ti_mcan_msg_handler;

void set_mcan_base_addr(uint32_t addr);
void set_filter_element(MCAN_StdMsgIDFilterElement *arr, uint8_t len);
void ti_mcan_register_interrupt(uint32_t iniNum);
void ti_mcan_enable_interrupt(void);
void ti_mcan_isr(void *arg);
void ti_mcan_config(Bool enableInternalLpbk);
void ti_mcan_send_msg(uint32_t id, uint32_t mcan_data_size, uint8_t *data);
void ti_mcan_register_handler(ti_mcan_msg_handler *handler, uint32_t length);
void ti_mcan_parse_handler();

#ifdef __cplusplus
}
#endif

#endif