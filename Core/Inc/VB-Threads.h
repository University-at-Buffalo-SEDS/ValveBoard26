#pragma once
#include "telemetry.h"
#include "tx_api.h"

#define ABORT_FLAG  (0x01U)
#define LAUNCH_FLAG (0x02U)

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;

void telemetry_thread_entry(ULONG initial_input);
UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool);
/* ------ Telemetry Thread ------ */

void main_thread_entry(ULONG initial_input);
UINT create_main_thread(TX_BYTE_POOL *byte_pool);

void safety_thread_entry(ULONG initial_input);
UINT create_safety_thread(TX_BYTE_POOL *byte_pool);

extern TX_EVENT_FLAGS_GROUP event_flags;
extern TX_QUEUE tx_queue;
